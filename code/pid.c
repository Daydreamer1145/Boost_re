#include "pid.h"

void PID_Init(PID*pid,float Kp,float Ki,float Kd,float outMax,float outMin,
		float sampleTime,PID_Direction controllerDirection)   //初始化PID结构体
{
	pid->controllerDirection=controllerDirection;   //设置PID的控制方向
	pid->Input=0.0;                                 //初始化测量值
	pid->lastError=0.0;                             //初始化上一次测量值
	pid->Output=0.0;                                //初始化输出值
	pid->Error=0.0;                                 //初始化误差值
	pid->Setpoint=0.0;
    pid->iTerm=0.0;                                 //初始化积分项

    if(sampleTime > 0.0f)                           //设置PID采样周期
    {
        pid->sampleTime = sampleTime;               
    }
    else
    {
        pid->sampleTime = 1.0f;                     //采样周期不能为0，若为0则设置为1.0f，防止后面计算微分系数时除以零
    }
	
	if(outMin >= outMax)        //如果PID输出值的下限大于上限，则直接return，跳出初始化函数
    {
        return;
    }

    pid->outMin = outMin;       //设置PID输出值的下限
    pid->outMax = outMax;       //设置PID输出值的上限
	 
    
    pid->Output = CONSTRAIN(pid->Output, outMin, outMax);  //对PID输出值进行限制，防止溢出
    pid->iTerm  = CONSTRAIN(pid->iTerm, outMin, outMax);   //对积分累加器进行限制，防止积分饱和
    
	
	PID_alter(pid,Kp,Ki,Kd);   //
	
    
}



void PID_Compute(PID*pid, float input)
{
	
	pid->Input = input;    //保存当前输入值
	
	
	pid->Error= pid->Setpoint- pid->Input;  //当前误差值，e(K)=目标值-测量值
	
	
	pid->iTerm += pid->alteredKi * pid->Error;  //积分运算
	

	pid->iTerm=CONSTRAIN(pid->iTerm, pid->outMin, pid->outMax);   //积分限幅，防止积分饱和
	
	
	pid->Output = pid->alteredKp * pid->Error + pid->iTerm - pid->alteredKd*(pid->Error - pid->lastError);//pid输出
    //比例项：`alteredKp * Error`
    //积分项：`iTerm`
    //微分项：`-alteredKd * (Error - lastError)`
    //值得注意的是，这里的微分项前面是负号，这可能是设计需要，也可能是符号写反了，需要结合 alteredKd 的定义及系统控制方向确认。

	pid->Output=CONSTRAIN(pid->Output,pid->outMin,pid->outMax);  //输出限幅
	pid->lastError=pid->Error; //保存当前误差值，作为下一次计算微分项的上一次误差值
}


void PID_alter(PID*pid,float Kp,float Ki,float Kd)   //更新PID参数
{
	   if(Kp < 0.0f || Ki < 0.0f || Kd < 0.0f)   //如果Kp、Ki、Kd有一个小于0，则直接return，跳出函数
    {
        return;
    }
	pid->Kp=Kp;    //设置PID的Kp、Ki、Kd
	pid->Ki=Ki;
	pid->Kd=Kd;
	
	
	pid->alteredKp = Kp;                     //经过进一步计算的Kp,Ki,Kd
    pid->alteredKi = Ki * pid->sampleTime;
    pid->alteredKd = Kd / pid->sampleTime;
	
	  if(pid->controllerDirection == Left)   //反方向
    {
        pid->alteredKp = -(pid->alteredKp);
        pid->alteredKi = -(pid->alteredKi);
        pid->alteredKd = -(pid->alteredKd);
    }
	

}

static unsigned int soft_start_counter = 0U; // 保存软启动进度，函数返回后仍保留当前步数

void soft_start_reset(void)                  // 提供显式复位接口，供停机、故障或重新使能时调用
{                                            // 软启动复位函数开始
    soft_start_counter = 0U;                 // 清零进度，使下一次正PWM指令从起始上限重新爬升
}                                            // 软启动复位函数结束

int soft_start(float output)                 // 对目标PWM比较值进行渐升限幅并转换为整数
{                                            // 软启动函数开始
    float current_max_pwm_value;             // 保存本次调用允许输出的最大PWM比较值
    int rounded_output;                      // 保存四舍五入后的整数PWM比较值

    if (!(output > 0.0f))                    // 非正数或NaN均按停机处理，避免异常值被转换成整数
    {                                        // 停机处理分支开始
        soft_start_reset();                  // 停机时同步清零进度，保证下次启动仍有软启动过程
        return 0;                            // 立即返回零，避免旧代码强制输出最小PWM而无法关断
    }                                        // 停机处理分支结束

    current_max_pwm_value = SOFT_START_MIN_PWM + // 从配置的起始PWM上限开始计算本次限幅值
                            (SOFT_START_MAX_PWM - SOFT_START_MIN_PWM) * // 计算整个软启动需要跨越的PWM范围
                            ((float)soft_start_counter / (float)SOFT_START_STEPS); // 按当前步数进行线性插值

    if (output > current_max_pwm_value)      // 判断控制器要求值是否超过当前软启动上限
    {                                        // 上限处理分支开始
        output = current_max_pwm_value;      // 仅压低过大的指令，不把较小的安全指令强制抬高
    }                                        // 上限处理分支结束

    if (output > SOFT_START_MAX_PWM)         // 判断输入是否超过配置的最终安全上限
    {                                        // 最终安全限幅分支开始
        output = SOFT_START_MAX_PWM;         // 将异常过大的输入限制在最终安全上限内
    }                                        // 最终安全限幅分支结束

    if (soft_start_counter < SOFT_START_STEPS) // 判断软启动是否仍处在渐升阶段
    {                                          // 软启动计数分支开始
        soft_start_counter++;                  // 每次有效调用前进一步，到达总步数后保持不变
    }                                          // 软启动计数分支结束

    rounded_output = (int)(output + 0.5f);    // 正数加0.5后取整，实现PWM比较值的四舍五入
    return rounded_output;                    // 返回可直接写入定时器CCR寄存器的整数值
}                                            // 软启动函数结束

void  PIDSetpointSet(PID *pid, float setpoint) { pid->Setpoint = setpoint; }
