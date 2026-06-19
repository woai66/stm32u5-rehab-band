/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wireless_link.c
  * @brief   Wireless UART packet parser for wrist node data.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "wireless_link.h"

static uint8_t rx_buf[WIRELESS_WRIST_FRAME_SIZE];
static uint8_t rx_index;
static WirelessWristFrame_t latest_frame;
static uint8_t latest_valid;
static WirelessLinkStats_t link_stats;

static uint16_t ReadU16Le(const uint8_t *buf)
{
  return (uint16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

static int16_t ReadI16Le(const uint8_t *buf)
{
  return (int16_t)ReadU16Le(buf);
}

static uint32_t ReadU32Le(const uint8_t *buf)
{
  return ((uint32_t)buf[3] << 24) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[1] << 8) |
         (uint32_t)buf[0];
}

static uint8_t CalcChecksum(const uint8_t *buf, uint8_t len)
{
  uint8_t sum = 0U;

  for (uint8_t i = 0U; i < len; i++)
  {
    sum = (uint8_t)(sum + buf[i]);
  }
  return sum;
}

static void DecodeFrame(const uint8_t *buf, WirelessWristFrame_t *frame)
{
  frame->seq = ReadU16Le(&buf[1]);
  frame->tick = ReadU32Le(&buf[3]);
  frame->q_x10000[0] = ReadI16Le(&buf[7]);
  frame->q_x10000[1] = ReadI16Le(&buf[9]);
  frame->q_x10000[2] = ReadI16Le(&buf[11]);
  frame->q_x10000[3] = ReadI16Le(&buf[13]);
  frame->gyro_raw[0] = ReadI16Le(&buf[15]);
  frame->gyro_raw[1] = ReadI16Le(&buf[17]);
  frame->gyro_raw[2] = ReadI16Le(&buf[19]);
  frame->heart_rate = ReadU16Le(&buf[21]);
}

void WirelessLink_Init(void)
{
  rx_index = 0U;
  latest_valid = 0U;
  link_stats.rx_bytes = 0U;
  link_stats.frames_ok = 0U;
  link_stats.checksum_errors = 0U;
  link_stats.resync_count = 0U;
}

uint8_t WirelessLink_PushByte(uint8_t byte, WirelessWristFrame_t *frame)
{
  link_stats.rx_bytes++;

  if (rx_index == 0U)
  {
    if (byte != WIRELESS_FRAME_HEADER)
    {
      link_stats.resync_count++;
      return 0U;
    }
  }
  rx_buf[rx_index++] = byte;

  if (rx_index < WIRELESS_WRIST_FRAME_SIZE)
  {
    return 0U;
  }

  rx_index = 0U;
  if (CalcChecksum(rx_buf, WIRELESS_WRIST_FRAME_SIZE - 1U) != rx_buf[WIRELESS_WRIST_FRAME_SIZE - 1U])
  {
    link_stats.checksum_errors++;
    return 0U;
  }

  DecodeFrame(rx_buf, &latest_frame);
  latest_valid = 1U;
  link_stats.frames_ok++;

  if (frame != NULL)
  {
    *frame = latest_frame;
  }
  return 1U;
}

uint8_t WirelessLink_GetLatest(WirelessWristFrame_t *frame)
{
  if ((latest_valid == 0U) || (frame == NULL))
  {
    return 0U;
  }

  *frame = latest_frame;
  return 1U;
}

void WirelessLink_GetStats(WirelessLinkStats_t *stats)
{
  if (stats != NULL)
  {
    *stats = link_stats;
  }
}
