#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Software-I2C pin configuration. Override these macros if pins differ. */
#ifndef OLED_SCL_GPIO_PORT
#define OLED_SCL_GPIO_PORT GPIOB
#endif

#ifndef OLED_SCL_GPIO_CLK_ENABLE
#define OLED_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

#ifndef OLED_SCL_GPIO_PIN
#define OLED_SCL_GPIO_PIN GPIO_PIN_6
#endif

#ifndef OLED_SDA_GPIO_PORT
#define OLED_SDA_GPIO_PORT GPIOB
#endif

#ifndef OLED_SDA_GPIO_CLK_ENABLE
#define OLED_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

#ifndef OLED_SDA_GPIO_PIN
#define OLED_SDA_GPIO_PIN GPIO_PIN_7
#endif

/* SSD1306 7-bit address 0x3C, shifted left for transmission. */
#ifndef OLED_I2C_WRITE_ADDRESS
#define OLED_I2C_WRITE_ADDRESS 0x78U
#endif

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t line, uint8_t column, char character);
void OLED_ShowString(uint8_t line, uint8_t column, const char *string);
void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
