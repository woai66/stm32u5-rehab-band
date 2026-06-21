/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wireless_link.h
 * @brief   Wireless UART packet codec for wrist node data.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __WIRELESS_LINK_H__
#define __WIRELESS_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define WIRELESS_FRAME_HEADER        (0xAAU)
#define WIRELESS_WRIST_FRAME_SIZE    (29U)

typedef struct
{
  uint16_t seq;
  uint32_t tick;
  int16_t acc_mg[3];
  int16_t gyro_dps_x10[3];
  int16_t angle_x100[3];
  uint16_t heart_rate;      /* bpm, 0 = invalid. */
  uint8_t status;           /* bit0: IMU valid; bit1: heart_rate valid. */
} WirelessWristFrame_t;

typedef struct
{
  uint32_t rx_bytes;
  uint32_t frames_ok;
  uint32_t checksum_errors;
  uint32_t resync_count;
} WirelessLinkStats_t;

void WirelessLink_Init(void);
uint8_t WirelessLink_BuildWristFrame(uint8_t *buf, const WirelessWristFrame_t *frame);
uint8_t WirelessLink_PushByte(uint8_t byte, WirelessWristFrame_t *frame);
uint8_t WirelessLink_GetLatest(WirelessWristFrame_t *frame);
void WirelessLink_GetStats(WirelessLinkStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* __WIRELESS_LINK_H__ */
