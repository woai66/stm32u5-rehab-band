#ifndef EMG_RF_MODEL_H
#define EMG_RF_MODEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  EMG_ACTION_ARM_FLEX_LIGHT = 0,
  EMG_ACTION_ARM_FLEX_STRONG = 1,
  EMG_ACTION_RELAX = 2,
  EMG_ACTION_SHAKE = 3
} EmgAction_t;

typedef struct
{
  uint16_t raw;
  int32_t baseline_x10;
  int32_t drop_x10;
  int32_t rectified_x10;
  int32_t envelope_x10;
  int32_t rms_x10;
} EmgModelInput_t;

EmgAction_t EmgRfModel_Predict(const EmgModelInput_t *input, uint16_t *confidence_x1000);
const char *EmgRfModel_LabelName(EmgAction_t action);

#ifdef __cplusplus
}
#endif

#endif /* EMG_RF_MODEL_H */
