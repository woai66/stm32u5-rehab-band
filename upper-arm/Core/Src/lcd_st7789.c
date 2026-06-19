#include "lcd_st7789.h"
#include "spi.h"
#include <string.h>

#define LCD_SPI_TIMEOUT_MS 100U
#define LCD_TX_BUF_SIZE    1024U

static uint8_t lcd_tx_buf[LCD_TX_BUF_SIZE];
/**
 * @brief  选中LCD屏幕（拉低CS引脚）
 */
static void LCD_Select(void)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void LCD_Unselect(void)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void LCD_CommandMode(void)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
}

static void LCD_DataMode(void)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

static void LCD_Reset(void)
{
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}
/**
 * @brief  通过SPI发送字节数据
 * @param  data: 指向待发送数据缓冲区的指针
 * @param  len: 待发送数据的长度
 * @retval HAL状态枚举值
 */
static HAL_StatusTypeDef LCD_WriteBytes(const uint8_t *data, uint16_t len)
{
    return HAL_SPI_Transmit(&hspi3, (uint8_t *)data, len, LCD_SPI_TIMEOUT_MS);
}
/**
 * @brief  向LCD发送单字节命令
 * @param  cmd: 命令字节
 */
static void LCD_WriteCommand(uint8_t cmd)
{
    LCD_Select();
    LCD_CommandMode();
    (void)LCD_WriteBytes(&cmd, 1);
    LCD_Unselect();
}

/**
 * @brief  向LCD发送单字节数据
 * @param  data: 数据字节
 */
static void LCD_WriteData8(uint8_t data)
{
    LCD_Select();
    LCD_DataMode();
    (void)LCD_WriteBytes(&data, 1);
    LCD_Unselect();
}
/**
 * @brief  向LCD发送多字节数据
 * @param  data: 指向数据缓冲区的指针
 * @param  len: 数据长度
 */
static void LCD_WriteData(const uint8_t *data, uint16_t len)
{
    LCD_Select();
    LCD_DataMode();
    (void)LCD_WriteBytes(data, len);
    LCD_Unselect();
}

/**
 * @brief  设置LCD显存写入地址范围
 * @param  x0: 起始X坐标
 * @param  y0: 起始Y坐标
 * @param  x1: 结束X坐标
 * @param  y1: 结束Y坐标
 * @note   自动处理坐标越界情况，并发送列地址设置(0x2A)、行地址设置(0x2B)及内存写命令(0x2C)
 */
static void LCD_SetAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    if (x0 >= LCD_WIDTH) {
        x0 = LCD_WIDTH - 1U;
    }
    if (x1 >= LCD_WIDTH) {
        x1 = LCD_WIDTH - 1U;
    }
    if (y0 >= LCD_HEIGHT) {
        y0 = LCD_HEIGHT - 1U;
    }
    if (y1 >= LCD_HEIGHT) {
        y1 = LCD_HEIGHT - 1U;
    }

    LCD_WriteCommand(0x2A);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)x0;
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)x1;
    LCD_WriteData(data, sizeof(data));

    LCD_WriteCommand(0x2B);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)y0;
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)y1;
    LCD_WriteData(data, sizeof(data));

    LCD_WriteCommand(0x2C);
}
/**
 * @brief  控制LCD背光开关
 * @param  on: 非0值开启背光，0值关闭背光
 */
void LCD_SetBacklight(uint8_t on)
{
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/**
 * @brief  初始化LCD屏幕
 * @note   执行硬件复位，发送ST7789初始化序列配置参数，最后清屏并开启背光
 */
void LCD_Init(void)
{
    LCD_Unselect();
    LCD_CommandMode();
    LCD_SetBacklight(0);
    LCD_Reset();

    LCD_WriteCommand(0x11);
    HAL_Delay(120);

    LCD_WriteCommand(0x36);
    LCD_WriteData8(0x00);

    LCD_WriteCommand(0x3A);
    LCD_WriteData8(0x05);

    LCD_WriteCommand(0xB2);
    LCD_WriteData8(0x0C);
    LCD_WriteData8(0x0C);
    LCD_WriteData8(0x00);
    LCD_WriteData8(0x33);
    LCD_WriteData8(0x33);

    LCD_WriteCommand(0xB7);
    LCD_WriteData8(0x35);

    LCD_WriteCommand(0xBB);
    LCD_WriteData8(0x32);

    LCD_WriteCommand(0xC2);
    LCD_WriteData8(0x01);

    LCD_WriteCommand(0xC3);
    LCD_WriteData8(0x15);

    LCD_WriteCommand(0xC4);
    LCD_WriteData8(0x20);

    LCD_WriteCommand(0xC6);
    LCD_WriteData8(0x0F);

    LCD_WriteCommand(0xD0);
    LCD_WriteData8(0xA4);
    LCD_WriteData8(0xA1);

    LCD_WriteCommand(0xE0);
    {
        const uint8_t p_gamma[] = {0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34};
        LCD_WriteData(p_gamma, sizeof(p_gamma));
    }

    LCD_WriteCommand(0xE1);
    {
        const uint8_t n_gamma[] = {0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34};
        LCD_WriteData(n_gamma, sizeof(n_gamma));
    }

    LCD_WriteCommand(0x21);
    LCD_WriteCommand(0x29);
    HAL_Delay(20);

    LCD_FillScreen(LCD_COLOR_BLACK);
    LCD_SetBacklight(1);
}
/**
 * @brief  在指定矩形区域内填充颜色
 * @param  x0: 起始X坐标
 * @param  y0: 起始Y坐标
 * @param  x1: 结束X坐标
 * @param  y1: 结束Y坐标
 * @param  color: 填充颜色 (RGB565格式)
 * @note   使用内部缓冲区分批发送数据以优化大量像素写入性能
 */
void LCD_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    uint32_t pixels;
    uint32_t bytes_left;
    uint16_t chunk;

    if ((x0 >= LCD_WIDTH) || (y0 >= LCD_HEIGHT)) {
        return;
    }
    if (x1 >= LCD_WIDTH) {
        x1 = LCD_WIDTH - 1U;
    }
    if (y1 >= LCD_HEIGHT) {
        y1 = LCD_HEIGHT - 1U;
    }
    if ((x1 < x0) || (y1 < y0)) {
        return;
    }

    pixels = ((uint32_t)(x1 - x0 + 1U)) * ((uint32_t)(y1 - y0 + 1U));
    bytes_left = pixels * 2U;

    for (uint16_t i = 0; i < LCD_TX_BUF_SIZE; i += 2U) {
        lcd_tx_buf[i] = (uint8_t)(color >> 8);
        lcd_tx_buf[i + 1U] = (uint8_t)color;
    }

    LCD_SetAddress(x0, y0, x1, y1);
    LCD_Select();
    LCD_DataMode();
    while (bytes_left > 0U) {
        chunk = (bytes_left > LCD_TX_BUF_SIZE) ? LCD_TX_BUF_SIZE : (uint16_t)bytes_left;
        (void)LCD_WriteBytes(lcd_tx_buf, chunk);
        bytes_left -= chunk;
    }
    LCD_Unselect();
}
/**
 * @brief  全屏填充指定颜色
 * @param  color: 填充颜色 (RGB565格式)
 */
void LCD_FillScreen(uint16_t color)
{
    LCD_Fill(0, 0, LCD_WIDTH - 1U, LCD_HEIGHT - 1U, color);
}
/**
 * @brief  在指定坐标绘制一个像素点
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  color: 像素颜色 (RGB565格式)
 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    uint8_t data[2];

    if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) {
        return;
    }

    data[0] = (uint8_t)(color >> 8);
    data[1] = (uint8_t)color;
    LCD_SetAddress(x, y, x, y);
    LCD_WriteData(data, sizeof(data));
}
/**
 * @brief  绘制空心矩形
 * @param  x: 左上角X坐标
 * @param  y: 左上角Y坐标
 * @param  w: 矩形宽度
 * @param  h: 矩形高度
 * @param  color: 边框颜色 (RGB565格式)
 * @note   通过绘制四条边线实现
 */
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0U) || (h == 0U)) {
        return;
    }

    LCD_Fill(x, y, x + w - 1U, y, color);
    LCD_Fill(x, y + h - 1U, x + w - 1U, y + h - 1U, color);
    LCD_Fill(x, y, x, y + h - 1U, color);
    LCD_Fill(x + w - 1U, y, x + w - 1U, y + h - 1U, color);
}