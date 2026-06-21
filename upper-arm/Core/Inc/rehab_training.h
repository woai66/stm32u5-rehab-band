#ifndef __REHAB_TRAINING_H__
#define __REHAB_TRAINING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  REHAB_TRAIN_ACTION_ELBOW_FLEX = 0,
  REHAB_TRAIN_ACTION_ARM_ABDUCTION,
  REHAB_TRAIN_ACTION_FOREARM_ROTATION,
  REHAB_TRAIN_ACTION_SHOULDER_LIFT
} RehabTrainingAction_t;

typedef struct
{
  RehabTrainingAction_t action;
  uint16_t target_reps;
  int32_t enter_angle_x100;
  int32_t exit_angle_x100;
  int32_t target_angle_x100;
} RehabTrainingConfig_t;

typedef struct
{
  RehabTrainingAction_t action;
  int32_t angle_x100;
  uint8_t emg_active;
  uint8_t valid;
  uint32_t tick_ms;
} RehabTrainingSample_t;

typedef struct
{
  RehabTrainingAction_t action;
  uint16_t target_reps;
  uint16_t reps;
  uint8_t active;
  uint8_t in_motion;
  uint8_t finished;
  uint8_t just_completed_rep;
  uint8_t just_finished;
  int32_t current_angle_x100;
  int32_t current_peak_x100;
  int32_t max_angle_x100;
  int32_t avg_peak_angle_x100;
  uint8_t emg_active;
  uint8_t emg_rate;
  uint8_t completion_score;
  uint8_t amplitude_score;
  uint8_t participation_score;
  uint8_t total_score;
} RehabTrainingState_t;

void RehabTraining_Init(void);
void RehabTraining_Start(const RehabTrainingConfig_t *config);
void RehabTraining_Stop(void);
void RehabTraining_Update(const RehabTrainingSample_t *sample);
void RehabTraining_GetState(RehabTrainingState_t *state);
const char *RehabTraining_ActionName(RehabTrainingAction_t action);

#ifdef __cplusplus
}
#endif

#endif /* __REHAB_TRAINING_H__ */
