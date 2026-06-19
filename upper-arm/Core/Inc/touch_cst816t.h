#ifndef __TOUCH_CST816T_H__
#define __TOUCH_CST816T_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CST816T_I2C_ADDR_8BIT 0x2AU

typedef struct {
    uint8_t detected;
    uint8_t gesture;
    uint8_t points;
    uint16_t x;
    uint16_t y;
} CST816T_TouchPoint_t;

HAL_StatusTypeDef CST816T_Init(void);
HAL_StatusTypeDef CST816T_ReadPoint(CST816T_TouchPoint_t *point);
uint8_t CST816T_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif