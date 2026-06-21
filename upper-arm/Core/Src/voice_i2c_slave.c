#include "voice_i2c_slave.h"
#include "i2c.h"

#define VOICE_I2C_HANDLE (&hi2c2)

static uint8_t voice_rx_byte;
static volatile uint8_t voice_latest_cmd;
static volatile uint8_t voice_has_cmd;
static volatile uint32_t voice_rx_count;
static volatile uint32_t voice_error_count;

HAL_StatusTypeDef VoiceI2C_Init(void)
{
  voice_rx_byte = 0U;
  voice_latest_cmd = 0U;
  voice_has_cmd = 0U;
  voice_rx_count = 0U;
  voice_error_count = 0U;

  return HAL_I2C_EnableListen_IT(VOICE_I2C_HANDLE);
}

uint8_t VoiceI2C_GetCommand(uint8_t *cmd)
{
  uint8_t has_cmd;

  if (cmd == NULL)
  {
    return 0U;
  }

  __disable_irq();
  has_cmd = voice_has_cmd;
  if (has_cmd != 0U)
  {
    *cmd = voice_latest_cmd;
    voice_has_cmd = 0U;
  }
  __enable_irq();

  return has_cmd;
}

void VoiceI2C_GetStatus(VoiceI2C_Status_t *status)
{
  if (status == NULL)
  {
    return;
  }

  __disable_irq();
  status->latest_cmd = voice_latest_cmd;
  status->has_cmd = voice_has_cmd;
  status->rx_count = voice_rx_count;
  status->error_count = voice_error_count;
  __enable_irq();
}

void VoiceI2C_HandleAddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transfer_direction)
{
  if (hi2c->Instance != I2C2)
  {
    return;
  }

  if (transfer_direction == I2C_DIRECTION_TRANSMIT)
  {
    (void)HAL_I2C_Slave_Seq_Receive_IT(hi2c, &voice_rx_byte, 1U, I2C_FIRST_AND_LAST_FRAME);
  }
}

void VoiceI2C_HandleRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2)
  {
    return;
  }

  voice_latest_cmd = voice_rx_byte;
  voice_has_cmd = 1U;
  voice_rx_count++;
}

void VoiceI2C_HandleListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2)
  {
    return;
  }

  (void)HAL_I2C_EnableListen_IT(hi2c);
}

void VoiceI2C_HandleErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2)
  {
    return;
  }

  voice_error_count++;
  (void)HAL_I2C_EnableListen_IT(hi2c);
}
