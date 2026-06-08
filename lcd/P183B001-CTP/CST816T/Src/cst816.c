#include "CST816T.h"
// IIC驱动实现
#include "math.h"

void CST816T_ReceiveByte(uint8_t Addr, uint8_t *Data);

void CST816T_Init(void)
{
    uint8_t ChipId    = 0;
    uint8_t FwVersion = 0;
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    CST816T_ReceiveByte(0xa7, &ChipId);
    // 读芯片id
    CST816T_ReceiveByte(0xa9, &FwVersion);
    // 读固件版本
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

void CST816T_SendByte(uint8_t Addr, uint8_t Data)
{
    uint8_t Buffer[2] = {Addr, Data};
    HAL_I2C_Master_Transmit(&hi2c3, 0x2A, Buffer, 2, HAL_MAX_DELAY);
}

void CST816T_ReceiveByte(uint8_t Addr, uint8_t *Data)
{
    HAL_I2C_Mem_Read(&hi2c3, 0x2a, Addr, I2C_MEMADD_SIZE_8BIT, Data, 1, HAL_MAX_DELAY);
}

uint8_t CST816_GetAction(uint16_t *X, uint16_t *Y, uint8_t *Gesture)
{
    uint8_t data[6];
back:
    HAL_I2C_Mem_Read(&hi2c3, 0x2a, 0x01, I2C_MEMADD_SIZE_8BIT, data, 6, 10);
    *Gesture = data[0];
    *X       = (uint16_t)((data[2] & 0x0F) << 8) | data[3];
    *Y       = (uint16_t)((data[4] & 0x0F) << 8) | data[5];
    if (*X > 250 || *Y > 250) {
        HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_RESET);
        osDelay(10);
        HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_SET);
        osDelay(50);
        return 0;
        goto back;
    }
    return data[1];
}