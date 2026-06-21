#ifndef __VOICE_I2C_SLAVE_H__
#define __VOICE_I2C_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  uint8_t latest_cmd;
  uint8_t has_cmd;
  uint32_t rx_count;
  uint32_t error_count;
} VoiceI2C_Status_t;

HAL_StatusTypeDef VoiceI2C_Init(void);
uint8_t VoiceI2C_GetCommand(uint8_t *cmd);
void VoiceI2C_GetStatus(VoiceI2C_Status_t *status);
void VoiceI2C_HandleAddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transfer_direction);
void VoiceI2C_HandleRxCpltCallback(I2C_HandleTypeDef *hi2c);
void VoiceI2C_HandleListenCpltCallback(I2C_HandleTypeDef *hi2c);
void VoiceI2C_HandleErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __VOICE_I2C_SLAVE_H__ */
