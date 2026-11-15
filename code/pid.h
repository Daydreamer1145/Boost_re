#ifndef __PID_H__
#define __PID_H__

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
//对PID的输出进行限制，防止过大或过小，保证送给 `TIM8->CCR1` 的值不会负数、不会超过 ARR（8400），避免 PWM 异常、炸功率管。
//逻辑等价
// if(x < lower)
//    return lower;
//else if(x > upper)
//    return upper;
//else
//   return x;

typedef enum
{
    Right,  //正向调节
    Left  //反向调节
} PID_Direction;
//  定义PID正向还是反向

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	
	float alteredKp;//经过进一步计算的Kp,Ki,Kd
	float alteredKi;
	float alteredKd;
	
	float Input;//测量值 
	float lastError;//上一次测量值
	
	float outMax;//输出限制值（防饱和）
	float outMin;
	
	float iTerm; //积分累加器（位置型PID）
	
	float Output;//输出值
	
	float Error;//误差值
	
	float Setpoint;//设置标准值
	
	float sampleTime;//PID采样周期
	

    PID_Direction controllerDirection;//控制方向

}PID;

#define SOFT_START_MIN_PWM      200.0f   // 定义软启动首次允许的最大PWM比较值，可按功率级需求调整
#define SOFT_START_MAX_PWM      4000.0f  // 定义软启动最终允许的最大PWM比较值，该值不得超过定时器ARR
#define SOFT_START_STEPS        200U     // 定义软启动从最小值升至最大值所需的函数调用次数
#if SOFT_START_STEPS == 0U               // 编译期检查软启动步数，避免线性插值时发生除零
#error "SOFT_START_STEPS must be greater than zero" // 配置错误时停止编译并给出明确提示
#endif                                   // 结束软启动步数的编译期检查

int soft_start(float output);             // 对PWM指令执行软启动限幅，并返回四舍五入后的比较值
void soft_start_reset(void);              // 复位软启动进度，使下一次使能时重新从起始值爬升

void PIDSetpointSet(PID *pid, float setpoint);  //设置PID的目标值


void PID_Init(PID*pid,float Kp,float Ki,float Kd,float outMax,float outMin,
		float sampleTime,PID_Direction controllerDirection); //PID初始化函数

void PID_Compute(PID*pid,float input);//PID计算函数
	

void PID_alter(PID*pid,float Kp,float Ki,float Kd);


#endif /* __PID_H__ */
