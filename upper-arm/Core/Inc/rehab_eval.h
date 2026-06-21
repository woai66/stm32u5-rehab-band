#ifndef __REHAB_EVAL_H__
#define __REHAB_EVAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint16_t target_reps;
  int32_t enter_angle_x100;
  int32_t exit_angle_x100;
  int32_t target_angle_x100;
} ElbowFlexEvalConfig_t;

typedef struct
{
  int32_t flex_angle_x100;
  uint8_t emg_active;
  uint8_t valid;
  uint32_t tick_ms;
} ElbowFlexEvalSample_t;

typedef struct
{
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
} ElbowFlexEvalState_t;

typedef struct
{
  uint16_t target_reps;
  int32_t enter_angle_x100;
  int32_t exit_angle_x100;
  int32_t target_angle_x100;
  int32_t enter_gyro_x10;
} ForearmRotEvalConfig_t;

typedef struct
{
  int32_t rotation_angle_x100;
  int32_t wrist_roll_gyro_x10;
  uint8_t upper_stable;
  uint8_t valid;
  uint32_t tick_ms;
} ForearmRotEvalSample_t;

typedef struct
{
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
  uint8_t completion_score;
  uint8_t amplitude_score;
  uint8_t total_score;
} ForearmRotEvalState_t;

typedef struct
{
  uint16_t target_reps;
  int32_t enter_angle_x100;
  int32_t exit_angle_x100;
  int32_t target_angle_x100;
  int32_t enter_gyro_x10;
} ArmAbdEvalConfig_t;

typedef struct
{
  int32_t abduction_angle_x100;
  int32_t upper_yaw_gyro_x10;
  uint8_t valid;
  uint32_t tick_ms;
} ArmAbdEvalSample_t;

typedef struct
{
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
  uint8_t completion_score;
  uint8_t amplitude_score;
  uint8_t total_score;
} ArmAbdEvalState_t;

typedef struct
{
  uint16_t target_reps;
  int32_t enter_angle_x100;
  int32_t exit_angle_x100;
  int32_t target_angle_x100;
  int32_t enter_gyro_x10;
} ShoulderLiftEvalConfig_t;

typedef struct
{
  int32_t lift_angle_x100;
  int32_t upper_pitch_gyro_x10;
  uint8_t valid;
  uint32_t tick_ms;
} ShoulderLiftEvalSample_t;

typedef struct
{
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
  uint8_t completion_score;
  uint8_t amplitude_score;
  uint8_t total_score;
} ShoulderLiftEvalState_t;

void ElbowFlexEval_DefaultConfig(ElbowFlexEvalConfig_t *config);
void ElbowFlexEval_Init(void);
void ElbowFlexEval_Start(const ElbowFlexEvalConfig_t *config);
void ElbowFlexEval_Stop(void);
void ElbowFlexEval_Update(const ElbowFlexEvalSample_t *sample);
void ElbowFlexEval_GetState(ElbowFlexEvalState_t *state);

void ForearmRotEval_DefaultConfig(ForearmRotEvalConfig_t *config);
void ForearmRotEval_Init(void);
void ForearmRotEval_Start(const ForearmRotEvalConfig_t *config);
void ForearmRotEval_Stop(void);
void ForearmRotEval_Update(const ForearmRotEvalSample_t *sample);
void ForearmRotEval_GetState(ForearmRotEvalState_t *state);

void ArmAbdEval_DefaultConfig(ArmAbdEvalConfig_t *config);
void ArmAbdEval_Init(void);
void ArmAbdEval_Start(const ArmAbdEvalConfig_t *config);
void ArmAbdEval_Stop(void);
void ArmAbdEval_Update(const ArmAbdEvalSample_t *sample);
void ArmAbdEval_GetState(ArmAbdEvalState_t *state);

void ShoulderLiftEval_DefaultConfig(ShoulderLiftEvalConfig_t *config);
void ShoulderLiftEval_Init(void);
void ShoulderLiftEval_Start(const ShoulderLiftEvalConfig_t *config);
void ShoulderLiftEval_Stop(void);
void ShoulderLiftEval_Update(const ShoulderLiftEvalSample_t *sample);
void ShoulderLiftEval_GetState(ShoulderLiftEvalState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* __REHAB_EVAL_H__ */
