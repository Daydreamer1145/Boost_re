#include "OLED.h"
#include "OLED_Font.h"

#define OLED_CONTROL_COMMAND 0x00U
#define OLED_CONTROL_DATA    0x40U
#define OLED_PAGE_COUNT      8U
#define OLED_WIDTH           128U
#define OLED_TEXT_ROWS       4U
#define OLED_TEXT_COLUMNS    16U

static void OLED_WriteSCL(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_GPIO_PIN, state);
}

static void OLED_WriteSDA(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_GPIO_PIN, state);
}

/* Short delay for the bit-banged I2C bus. */
static void OLED_I2C_Delay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < (SystemCoreClock / 3000000U); ++count)
    {
        __NOP();
    }
}

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    OLED_SCL_GPIO_CLK_ENABLE();
    OLED_SDA_GPIO_CLK_ENABLE();

    /* Set the output latches high before enabling open-drain mode. */
    OLED_WriteSCL(GPIO_PIN_SET);
    OLED_WriteSDA(GPIO_PIN_SET);

    gpio.Pin = OLED_SCL_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_SCL_GPIO_PORT, &gpio);

    gpio.Pin = OLED_SDA_GPIO_PIN;
    HAL_GPIO_Init(OLED_SDA_GPIO_PORT, &gpio);

    OLED_WriteSCL(GPIO_PIN_SET);
    OLED_WriteSDA(GPIO_PIN_SET);
}

static void OLED_I2C_Start(void)
{
    OLED_WriteSDA(GPIO_PIN_SET);
    OLED_WriteSCL(GPIO_PIN_SET);
    OLED_I2C_Delay();
    OLED_WriteSDA(GPIO_PIN_RESET);
    OLED_I2C_Delay();
    OLED_WriteSCL(GPIO_PIN_RESET);
}

static void OLED_I2C_Stop(void)
{
    OLED_WriteSDA(GPIO_PIN_RESET);
    OLED_I2C_Delay();
    OLED_WriteSCL(GPIO_PIN_SET);
    OLED_I2C_Delay();
    OLED_WriteSDA(GPIO_PIN_SET);
    OLED_I2C_Delay();
}

static void OLED_I2C_SendByte(uint8_t byte)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; ++bit)
    {
        OLED_WriteSDA((byte & (uint8_t)(0x80U >> bit)) != 0U
                          ? GPIO_PIN_SET
                          : GPIO_PIN_RESET);
        OLED_I2C_Delay();
        OLED_WriteSCL(GPIO_PIN_SET);
        OLED_I2C_Delay();
        OLED_WriteSCL(GPIO_PIN_RESET);
    }

    /* Release SDA during the ACK clock. ACK is not sampled by this driver. */
    OLED_WriteSDA(GPIO_PIN_SET);
    OLED_I2C_Delay();
    OLED_WriteSCL(GPIO_PIN_SET);
    OLED_I2C_Delay();
    OLED_WriteSCL(GPIO_PIN_RESET);
}

static void OLED_WriteBuffer(uint8_t control, const uint8_t *data, uint16_t length)
{
    uint16_t index;

    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_WRITE_ADDRESS);
    OLED_I2C_SendByte(control);

    for (index = 0U; index < length; ++index)
    {
        OLED_I2C_SendByte(data[index]);
    }

    OLED_I2C_Stop();
}

static void OLED_WriteCommand(uint8_t command)
{
    OLED_WriteBuffer(OLED_CONTROL_COMMAND, &command, 1U);
}

static void OLED_SetCursor(uint8_t page, uint8_t x)
{
    OLED_WriteCommand((uint8_t)(0xB0U | page));
    OLED_WriteCommand((uint8_t)(0x10U | ((x & 0xF0U) >> 4U)));
    OLED_WriteCommand((uint8_t)(x & 0x0FU));
}

static uint32_t OLED_Pow(uint32_t base, uint8_t exponent)
{
    uint32_t result = 1U;

    while (exponent-- > 0U)
    {
        result *= base;
    }

    return result;
}

void OLED_Clear(void)
{
    static const uint8_t zeros[OLED_WIDTH] = {0U};
    uint8_t page;

    for (page = 0U; page < OLED_PAGE_COUNT; ++page)
    {
        OLED_SetCursor(page, 0U);
        OLED_WriteBuffer(OLED_CONTROL_DATA, zeros, OLED_WIDTH);
    }
}

void OLED_ShowChar(uint8_t line, uint8_t column, char character)
{
    uint8_t glyph;
    uint8_t page;
    uint8_t x;

    if ((line < 1U) || (line > OLED_TEXT_ROWS) ||
        (column < 1U) || (column > OLED_TEXT_COLUMNS) ||
        (character < ' ') || (character > '~'))
    {
        return;
    }

    glyph = (uint8_t)(character - ' ');
    page = (uint8_t)((line - 1U) * 2U);
    x = (uint8_t)((column - 1U) * 8U);

    OLED_SetCursor(page, x);
    OLED_WriteBuffer(OLED_CONTROL_DATA,
                     &OLED_F8x16[(uint16_t)glyph * 16U], 8U);
    OLED_SetCursor((uint8_t)(page + 1U), x);
    OLED_WriteBuffer(OLED_CONTROL_DATA,
                     &OLED_F8x16[((uint16_t)glyph * 16U) + 8U], 8U);
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *string)
{
    if (string == NULL)
    {
        return;
    }

    while ((*string != '\0') && (column <= OLED_TEXT_COLUMNS))
    {
        OLED_ShowChar(line, column, *string);
        ++column;
        ++string;
    }
}

void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t index;

    if ((length == 0U) || (length > 10U))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        uint32_t divisor = OLED_Pow(10U, (uint8_t)(length - index - 1U));
        char digit = (char)(((number / divisor) % 10U) + (uint32_t)'0');
        OLED_ShowChar(line, (uint8_t)(column + index), digit);
    }
}

void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
    uint32_t magnitude;

    if ((length == 0U) || (length > 10U))
    {
        return;
    }

    if (number < 0)
    {
        OLED_ShowChar(line, column, '-');
        magnitude = 0U - (uint32_t)number;
    }
    else
    {
        OLED_ShowChar(line, column, '+');
        magnitude = (uint32_t)number;
    }

    OLED_ShowNum(line, (uint8_t)(column + 1U), magnitude, length);
}

void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t index;

    if ((length == 0U) || (length > 8U))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        uint32_t divisor = OLED_Pow(16U, (uint8_t)(length - index - 1U));
        uint8_t digit = (uint8_t)((number / divisor) % 16U);
        char character = (digit < 10U)
                             ? (char)(digit + (uint8_t)'0')
                             : (char)(digit - 10U + (uint8_t)'A');
        OLED_ShowChar(line, (uint8_t)(column + index), character);
    }
}

void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t index;

    if ((length == 0U) || (length > 32U))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        uint32_t divisor = OLED_Pow(2U, (uint8_t)(length - index - 1U));
        char digit = (char)(((number / divisor) % 2U) + (uint32_t)'0');
        OLED_ShowChar(line, (uint8_t)(column + index), digit);
    }
}

void OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU,       /* Display off. */
        0xD5U, 0x80U,
        0xA8U, 0x3FU,
        0xD3U, 0x00U,
        0x40U,
        0xA1U,
        0xC8U,
        0xDAU, 0x12U,
        0x81U, 0xCFU,
        0xD9U, 0xF1U,
        0xDBU, 0x30U,
        0xA4U,
        0xA6U,
        0x8DU, 0x14U,
        0xAFU        /* Display on. */
    };

    OLED_GPIO_Init();
    HAL_Delay(100U);
    OLED_WriteBuffer(OLED_CONTROL_COMMAND, init_commands,
                     (uint16_t)sizeof(init_commands));
    OLED_Clear();
}
