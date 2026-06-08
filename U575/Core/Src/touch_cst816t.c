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

static HAL_StatusTypeDef CST816T_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2C_ADDR_8BIT, data, sizeof(data), CST816T_TIMEOUT_MS);
}

static HAL_StatusTypeDef CST816T_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, CST816T_I2C_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, data, len, CST816T_TIMEOUT_MS);
}

static void CST816T_Reset(void)
{
    HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

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
        point->detected = 1U;
    }

    return HAL_OK;
}

uint8_t CST816T_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(TP_INT_GPIO_Port, TP_INT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}