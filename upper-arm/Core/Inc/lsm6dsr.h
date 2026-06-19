#ifndef __LSM6DSR_H__
#define __LSM6DSR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  float acc_g_x;
  float acc_g_y;
  float acc_g_z;
  float gyro_dps_x;
  float gyro_dps_y;
  float gyro_dps_z;
} LSM6DSR_Data_t;

HAL_StatusTypeDef LSM6DSR_Init(void);
HAL_StatusTypeDef LSM6DSR_ReadRaw(LSM6DSR_Data_t *data);
HAL_StatusTypeDef LSM6DSR_ReadRawDma(LSM6DSR_Data_t *data, uint32_t timeout_ms);
void LSM6DSR_PrintRaw(const LSM6DSR_Data_t *data);
uint8_t LSM6DSR_ReadWhoAmI(void);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DSR_H__ */
