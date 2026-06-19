#ifndef __EMG_SENSOR_H__
#define __EMG_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stddef.h>
#include <stdint.h>

#define EMG_SENSOR_BUFFER_LEN           (64U)
#define EMG_SENSOR_SAMPLE_RATE_HZ       (500U)
#define EMG_SENSOR_DEFAULT_ACTIVE_DELTA (120.0f)
#define EMG_SENSOR_DEFAULT_FULL_SCALE_DELTA (1200.0f)

typedef struct
{
  uint16_t raw;
  float baseline;
  float noise_level;
  float rectified;
  float envelope;
  float rms;
  float voltage_mv;
  uint8_t active;
  uint8_t strength;
  uint8_t participation;
  uint8_t calibrated;
} EmgSensor_State_t;

typedef struct
{
  uint16_t raw;
  int32_t baseline_x10;
  int32_t drop_x10;
  int32_t rectified_x10;
  int32_t envelope_x10;
  int32_t rms_x10;
  uint8_t active;
  uint8_t strength;
  uint8_t calibrated;
} EmgSensor_Features_t;

HAL_StatusTypeDef EmgSensor_Start(void);
void EmgSensor_ProcessHalfBuffer(void);
void EmgSensor_ProcessFullBuffer(void);
void EmgSensor_GetState(EmgSensor_State_t *state);
void EmgSensor_GetFeatures(EmgSensor_Features_t *features);
void EmgSensor_SetThreshold(float active_delta, float full_scale_delta);
int EmgSensor_FormatDebugLine(char *buffer, size_t buffer_len);

#ifdef __cplusplus
}
#endif

#endif /* __EMG_SENSOR_H__ */
