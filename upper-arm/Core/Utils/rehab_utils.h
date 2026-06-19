#ifndef __REHAB_UTILS_H__
#define __REHAB_UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float alpha;
  float value;
  uint8_t initialized;
} Rehab_FirstOrderFilter_t;

typedef struct
{
  float *buffer;
  uint16_t length;
  uint16_t index;
  uint16_t count;
  float sum;
} Rehab_MovingAverage_t;

float Rehab_ClampFloat(float value, float min_value, float max_value);
int32_t Rehab_ClampInt32(int32_t value, int32_t min_value, int32_t max_value);
float Rehab_MapFloat(float value,
                     float in_min,
                     float in_max,
                     float out_min,
                     float out_max);
float Rehab_AbsFloat(float value);
float Rehab_DeadbandFloat(float value, float deadband);

void Rehab_FirstOrderFilterInit(Rehab_FirstOrderFilter_t *filter,
                                float alpha,
                                float initial_value);
float Rehab_FirstOrderFilterUpdate(Rehab_FirstOrderFilter_t *filter,
                                   float input);

void Rehab_MovingAverageInit(Rehab_MovingAverage_t *avg,
                             float *buffer,
                             uint16_t length);
float Rehab_MovingAverageUpdate(Rehab_MovingAverage_t *avg, float input);
float Rehab_RmsFloat(const float *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __REHAB_UTILS_H__ */
