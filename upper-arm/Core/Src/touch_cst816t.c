#include "touch_cst816t.h"
#include "i2c.h"
#include "lcd_st7789.h"
#include <string.h>

#define CST816T_REG_GESTURE_ID 0x01U
#define CST816T_REG_FINGER_NUM 0x02U
#define CST816T_REG_CHIP_ID    0xA7U
#define CST816T_REG_FW_VERSION 0xA9U
#define CST816T_REG_DIS_SLEEP  0xFEU
#define CST816T_TIMEOUT_MS     50U
/**
 * @brief 向CST816T触摸屏控制器写入寄存器数据
 * 
 * @param reg   寄存器地址
 * @param value 要写入的数据值
 * 
 * @return HAL_StatusTypeDef HAL_OK表示成功，其他表示失败
 */
static HAL_StatusTypeDef CST816T_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2C_ADDR_8BIT, data, sizeof(data), CST816T_TIMEOUT_MS);
}
/**
 * @brief 从CST816T触摸屏控制器读取寄存器数据
 * 
 * @param reg  寄存器地址
 * @param data 用于存储读取数据的缓冲区指针
 * @param len  要读取的数据长度
 * 
 * @return HAL_StatusTypeDef HAL_OK表示成功，其他表示失败
 */
static HAL_StatusTypeDef CST816T_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, CST816T_I2C_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, data, len, CST816T_TIMEOUT_MS);
}
/**
 * @brief 硬件复位CST816T触摸屏控制器
 * 
 * 通过拉低复位引脚并保持一段时间后拉高，完成硬件复位时序
 */
static void CST816T_Reset(void)
{
    HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}
/**
 * @brief 初始化CST816T触摸屏控制器
 * 
 * 执行硬件复位，读取芯片ID验证通信是否正常，读取固件版本（仅用于调试参考），
 * 并禁用睡眠模式以确保持续响应触摸事件。
 * 
 * @return HAL_StatusTypeDef HAL_OK表示初始化成功，其他表示失败
 */
HAL_StatusTypeDef CST816T_Init(void)
{
    uint8_t id = 0;
    uint8_t fw = 0;
    HAL_StatusTypeDef status;

    CST816T_Reset();

    status = CST816T_ReadReg(CST816T_REG_CHIP_ID, &id, 1);
    if (status != HAL_OK) {
        return status;
    }

    (void)CST816T_ReadReg(CST816T_REG_FW_VERSION, &fw, 1);
    (void)fw;

    (void)CST816T_WriteReg(CST816T_REG_DIS_SLEEP, 0x01U);
    return HAL_OK;
}
/**
 * @brief 读取当前触摸点信息
 * 
 * 从CST816T读取手势ID、触摸点数量及第一个触摸点的坐标信息。
 * 如果检测到有效触摸点且坐标在LCD屏幕范围内，则标记为已检测。
 * 
 * @param point 指向CST816T_TouchPoint_t结构体的指针，用于存储触摸点信息
 * 
 * @return HAL_StatusTypeDef HAL_OK表示读取成功，HAL_ERROR表示参数无效，其他表示I2C通信失败
 */
HAL_StatusTypeDef CST816T_ReadPoint(CST816T_TouchPoint_t *point)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    if (point == NULL) {
        return HAL_ERROR;
    }

    memset(point, 0, sizeof(*point));
    status = CST816T_ReadReg(CST816T_REG_GESTURE_ID, data, sizeof(data));
    if (status != HAL_OK) {
        return status;
    }

    point->gesture = data[0];
    point->points = data[1] & 0x0FU;
    point->x = (uint16_t)(((uint16_t)(data[2] & 0x0FU) << 8) | data[3]);
    point->y = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);

    if ((point->points > 0U) && (point->x < LCD_WIDTH) && (point->y < LCD_HEIGHT)) {
        /* 芯片原始 Y 与 LCD 坐标系上下相反，翻转后再对齐显示 */
        point->y = (uint16_t)(LCD_HEIGHT - 1U - point->y);
        point->detected = 1U;
    }

    return HAL_OK;
}
/**
 * @brief 检查触摸屏是否被按下
 * 
 * 通过读取中断引脚电平状态判断是否有触摸事件发生。
 * 低电平表示有触摸，高电平表示无触摸。
 * 
 * @return uint8_t 1表示被按下，0表示未被按下
 */
uint8_t CST816T_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(TP_INT_GPIO_Port, TP_INT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}