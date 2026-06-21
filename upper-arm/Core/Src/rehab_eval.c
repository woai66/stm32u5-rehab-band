#include "rehab_eval.h"

#include <string.h>

typedef struct
{
  ElbowFlexEvalConfig_t config;
  ElbowFlexEvalState_t state;
  uint32_t peak_sum_x100;
  uint16_t emg_active_reps;
  uint8_t current_rep_emg;
} ElbowFlexEvalContext_t;

typedef struct
{
  ForearmRotEvalConfig_t config;
  ForearmRotEvalState_t state;
  uint32_t peak_sum_x100;
} ForearmRotEvalContext_t;

typedef struct
{
  ArmAbdEvalConfig_t config;
  ArmAbdEvalState_t state;
  uint32_t peak_sum_x100;
} ArmAbdEvalContext_t;

typedef struct
{
  ShoulderLiftEvalConfig_t config;
  ShoulderLiftEvalState_t state;
  uint32_t peak_sum_x100;
} ShoulderLiftEvalContext_t;

static ElbowFlexEvalContext_t elbow_ctx;
static ForearmRotEvalContext_t forearm_ctx;
static ArmAbdEvalContext_t arm_abd_ctx;
static ShoulderLiftEvalContext_t shoulder_lift_ctx;

static uint8_t ClampPercentU32(uint32_t value)
{
  return (value > 100U) ? 100U : (uint8_t)value;
}

static uint8_t ScoreFromRatioU32(uint32_t value, uint32_t target)
{
  if (target == 0U)
  {
    return 0U;
  }

  return ClampPercentU32((value * 100U) / target);
}

static int32_t AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static void SanitizeElbowConfig(ElbowFlexEvalConfig_t *config)
{
  if (config->target_reps == 0U)
  {
    config->target_reps = 5U;
  }
  if (config->enter_angle_x100 <= 0)
  {
    config->enter_angle_x100 = 3000;
  }
  if (config->exit_angle_x100 <= 0)
  {
    config->exit_angle_x100 = 3000;
  }
  if (config->target_angle_x100 <= 0)
  {
    config->target_angle_x100 = 15000;
  }
  if (config->exit_angle_x100 > config->enter_angle_x100)
  {
    config->exit_angle_x100 = config->enter_angle_x100;
  }
}

static void SanitizeForearmConfig(ForearmRotEvalConfig_t *config)
{
  if (config->target_reps == 0U)
  {
    config->target_reps = 5U;
  }
  if (config->enter_angle_x100 <= 0)
  {
    config->enter_angle_x100 = 1500;
  }
  if (config->exit_angle_x100 <= 0)
  {
    config->exit_angle_x100 = 800;
  }
  if (config->target_angle_x100 <= 0)
  {
    config->target_angle_x100 = 6000;
  }
  if (config->enter_gyro_x10 <= 0)
  {
    config->enter_gyro_x10 = 200;
  }
  if (config->exit_angle_x100 > config->enter_angle_x100)
  {
    config->exit_angle_x100 = config->enter_angle_x100 / 2;
  }
}

static void SanitizeArmAbdConfig(ArmAbdEvalConfig_t *config)
{
  if (config->target_reps == 0U)
  {
    config->target_reps = 5U;
  }
  if (config->enter_angle_x100 <= 0)
  {
    config->enter_angle_x100 = 2000;
  }
  if (config->exit_angle_x100 <= 0)
  {
    config->exit_angle_x100 = 1800;
  }
  if (config->target_angle_x100 <= 0)
  {
    config->target_angle_x100 = 9000;
  }
  if (config->enter_gyro_x10 <= 0)
  {
    config->enter_gyro_x10 = 250;
  }
  if (config->exit_angle_x100 > config->enter_angle_x100)
  {
    config->exit_angle_x100 = config->enter_angle_x100 / 2;
  }
}

static void SanitizeShoulderLiftConfig(ShoulderLiftEvalConfig_t *config)
{
  if (config->target_reps == 0U)
  {
    config->target_reps = 5U;
  }
  if (config->enter_angle_x100 <= 0)
  {
    config->enter_angle_x100 = 1800;
  }
  if (config->exit_angle_x100 <= 0)
  {
    config->exit_angle_x100 = 1000;
  }
  if (config->target_angle_x100 <= 0)
  {
    config->target_angle_x100 = 9000;
  }
  if (config->enter_gyro_x10 <= 0)
  {
    config->enter_gyro_x10 = 80;
  }
  if (config->exit_angle_x100 > config->enter_angle_x100)
  {
    config->exit_angle_x100 = config->enter_angle_x100 / 2;
  }
}

static void UpdateElbowScores(void)
{
  uint32_t completion;
  uint32_t amplitude;
  uint32_t participation;
  uint32_t total;

  completion = ScoreFromRatioU32(elbow_ctx.state.reps, elbow_ctx.config.target_reps);
  amplitude = ScoreFromRatioU32((uint32_t)elbow_ctx.state.max_angle_x100,
                                (uint32_t)elbow_ctx.config.target_angle_x100);
  participation = (elbow_ctx.state.reps == 0U) ?
      0U : ScoreFromRatioU32(elbow_ctx.emg_active_reps, elbow_ctx.state.reps);

  total = (completion * 40U + amplitude * 40U + participation * 20U) / 100U;

  elbow_ctx.state.completion_score = (uint8_t)completion;
  elbow_ctx.state.amplitude_score = (uint8_t)amplitude;
  elbow_ctx.state.participation_score = (uint8_t)participation;
  elbow_ctx.state.total_score = (uint8_t)total;
  elbow_ctx.state.emg_rate = (uint8_t)participation;
}

static void UpdateForearmScores(void)
{
  uint32_t completion;
  uint32_t amplitude;
  uint32_t total;

  completion = ScoreFromRatioU32(forearm_ctx.state.reps, forearm_ctx.config.target_reps);
  amplitude = ScoreFromRatioU32((uint32_t)forearm_ctx.state.max_angle_x100,
                                (uint32_t)forearm_ctx.config.target_angle_x100);
  total = (completion * 50U + amplitude * 50U) / 100U;

  forearm_ctx.state.completion_score = (uint8_t)completion;
  forearm_ctx.state.amplitude_score = (uint8_t)amplitude;
  forearm_ctx.state.total_score = (uint8_t)total;
}

static void UpdateArmAbdScores(void)
{
  uint32_t completion;
  uint32_t amplitude;
  uint32_t total;

  completion = ScoreFromRatioU32(arm_abd_ctx.state.reps, arm_abd_ctx.config.target_reps);
  amplitude = ScoreFromRatioU32((uint32_t)arm_abd_ctx.state.max_angle_x100,
                                (uint32_t)arm_abd_ctx.config.target_angle_x100);
  total = (completion * 50U + amplitude * 50U) / 100U;

  arm_abd_ctx.state.completion_score = (uint8_t)completion;
  arm_abd_ctx.state.amplitude_score = (uint8_t)amplitude;
  arm_abd_ctx.state.total_score = (uint8_t)total;
}

static void UpdateShoulderLiftScores(void)
{
  uint32_t completion;
  uint32_t amplitude;
  uint32_t total;

  completion = ScoreFromRatioU32(shoulder_lift_ctx.state.reps,
                                 shoulder_lift_ctx.config.target_reps);
  amplitude = ScoreFromRatioU32((uint32_t)shoulder_lift_ctx.state.max_angle_x100,
                                (uint32_t)shoulder_lift_ctx.config.target_angle_x100);
  total = (completion * 50U + amplitude * 50U) / 100U;

  shoulder_lift_ctx.state.completion_score = (uint8_t)completion;
  shoulder_lift_ctx.state.amplitude_score = (uint8_t)amplitude;
  shoulder_lift_ctx.state.total_score = (uint8_t)total;
}

void ElbowFlexEval_DefaultConfig(ElbowFlexEvalConfig_t *config)
{
  if (config != 0)
  {
    config->target_reps = 5U;
    config->enter_angle_x100 = 3000;
    config->exit_angle_x100 = 3000;
    config->target_angle_x100 = 15000;
  }
}

void ElbowFlexEval_Init(void)
{
  memset(&elbow_ctx, 0, sizeof(elbow_ctx));
}

void ElbowFlexEval_Start(const ElbowFlexEvalConfig_t *config)
{
  ElbowFlexEvalConfig_t local_config;

  if (config == 0)
  {
    ElbowFlexEval_DefaultConfig(&local_config);
  }
  else
  {
    local_config = *config;
  }
  SanitizeElbowConfig(&local_config);

  memset(&elbow_ctx, 0, sizeof(elbow_ctx));
  elbow_ctx.config = local_config;
  elbow_ctx.state.target_reps = local_config.target_reps;
  elbow_ctx.state.active = 1U;
}

void ElbowFlexEval_Stop(void)
{
  elbow_ctx.state.active = 0U;
  elbow_ctx.state.in_motion = 0U;
}

void ElbowFlexEval_Update(const ElbowFlexEvalSample_t *sample)
{
  ElbowFlexEvalState_t *state = &elbow_ctx.state;

  if (sample == 0)
  {
    return;
  }

  state->just_completed_rep = 0U;
  state->just_finished = 0U;

  if ((state->active == 0U) || (state->finished != 0U))
  {
    return;
  }

  if (sample->valid == 0U)
  {
    state->current_angle_x100 = 0;
    state->emg_active = sample->emg_active;
    UpdateElbowScores();
    return;
  }

  state->current_angle_x100 = sample->flex_angle_x100;
  state->emg_active = sample->emg_active;

  if (state->in_motion == 0U)
  {
    if (sample->flex_angle_x100 >= elbow_ctx.config.enter_angle_x100)
    {
      state->in_motion = 1U;
      state->current_peak_x100 = sample->flex_angle_x100;
      elbow_ctx.current_rep_emg = sample->emg_active;
    }
  }
  else
  {
    if (sample->flex_angle_x100 > state->current_peak_x100)
    {
      state->current_peak_x100 = sample->flex_angle_x100;
    }
    if (sample->emg_active != 0U)
    {
      elbow_ctx.current_rep_emg = 1U;
    }

    if (sample->flex_angle_x100 <= elbow_ctx.config.exit_angle_x100)
    {
      state->in_motion = 0U;
      state->reps++;
      state->just_completed_rep = 1U;

      if (state->current_peak_x100 > state->max_angle_x100)
      {
        state->max_angle_x100 = state->current_peak_x100;
      }

      elbow_ctx.peak_sum_x100 += (uint32_t)state->current_peak_x100;
      state->avg_peak_angle_x100 = (int32_t)(elbow_ctx.peak_sum_x100 / (uint32_t)state->reps);

      if (elbow_ctx.current_rep_emg != 0U)
      {
        elbow_ctx.emg_active_reps++;
      }
      elbow_ctx.current_rep_emg = 0U;

      if (state->reps >= elbow_ctx.config.target_reps)
      {
        state->finished = 1U;
        state->active = 0U;
        state->just_finished = 1U;
      }
    }
  }

  UpdateElbowScores();
}

void ElbowFlexEval_GetState(ElbowFlexEvalState_t *state)
{
  if (state != 0)
  {
    *state = elbow_ctx.state;
  }
}

void ForearmRotEval_DefaultConfig(ForearmRotEvalConfig_t *config)
{
  if (config != 0)
  {
    config->target_reps = 5U;
    config->enter_angle_x100 = 1500;
    config->exit_angle_x100 = 800;
    config->target_angle_x100 = 6000;
    config->enter_gyro_x10 = 200;
  }
}

void ForearmRotEval_Init(void)
{
  memset(&forearm_ctx, 0, sizeof(forearm_ctx));
}

void ForearmRotEval_Start(const ForearmRotEvalConfig_t *config)
{
  ForearmRotEvalConfig_t local_config;

  if (config == 0)
  {
    ForearmRotEval_DefaultConfig(&local_config);
  }
  else
  {
    local_config = *config;
  }
  SanitizeForearmConfig(&local_config);

  memset(&forearm_ctx, 0, sizeof(forearm_ctx));
  forearm_ctx.config = local_config;
  forearm_ctx.state.target_reps = local_config.target_reps;
  forearm_ctx.state.active = 1U;
}

void ForearmRotEval_Stop(void)
{
  forearm_ctx.state.active = 0U;
  forearm_ctx.state.in_motion = 0U;
}

void ForearmRotEval_Update(const ForearmRotEvalSample_t *sample)
{
  ForearmRotEvalState_t *state = &forearm_ctx.state;

  if (sample == 0)
  {
    return;
  }

  state->just_completed_rep = 0U;
  state->just_finished = 0U;

  if ((state->active == 0U) || (state->finished != 0U))
  {
    return;
  }

  if ((sample->valid == 0U) || (sample->upper_stable == 0U))
  {
    state->current_angle_x100 = (sample->valid != 0U) ? sample->rotation_angle_x100 : 0;
    state->in_motion = 0U;
    state->current_peak_x100 = 0;
    UpdateForearmScores();
    return;
  }

  state->current_angle_x100 = sample->rotation_angle_x100;

  if (state->in_motion == 0U)
  {
    if ((sample->rotation_angle_x100 >= forearm_ctx.config.enter_angle_x100) ||
        (AbsI32(sample->wrist_roll_gyro_x10) >= forearm_ctx.config.enter_gyro_x10))
    {
      state->in_motion = 1U;
      state->current_peak_x100 = sample->rotation_angle_x100;
    }
  }
  else
  {
    if (sample->rotation_angle_x100 > state->current_peak_x100)
    {
      state->current_peak_x100 = sample->rotation_angle_x100;
    }

    if ((sample->rotation_angle_x100 <= forearm_ctx.config.exit_angle_x100) &&
        (state->current_peak_x100 >= forearm_ctx.config.enter_angle_x100))
    {
      state->in_motion = 0U;
      state->reps++;
      state->just_completed_rep = 1U;

      if (state->current_peak_x100 > state->max_angle_x100)
      {
        state->max_angle_x100 = state->current_peak_x100;
      }

      forearm_ctx.peak_sum_x100 += (uint32_t)state->current_peak_x100;
      state->avg_peak_angle_x100 = (int32_t)(forearm_ctx.peak_sum_x100 / (uint32_t)state->reps);

      if (state->reps >= forearm_ctx.config.target_reps)
      {
        state->finished = 1U;
        state->active = 0U;
        state->just_finished = 1U;
      }
    }
  }

  UpdateForearmScores();
}

void ForearmRotEval_GetState(ForearmRotEvalState_t *state)
{
  if (state != 0)
  {
    *state = forearm_ctx.state;
  }
}

void ArmAbdEval_DefaultConfig(ArmAbdEvalConfig_t *config)
{
  if (config != 0)
  {
    config->target_reps = 5U;
    config->enter_angle_x100 = 2000;
    config->exit_angle_x100 = 1800;
    config->target_angle_x100 = 9000;
    config->enter_gyro_x10 = 250;
  }
}

void ArmAbdEval_Init(void)
{
  memset(&arm_abd_ctx, 0, sizeof(arm_abd_ctx));
}

void ArmAbdEval_Start(const ArmAbdEvalConfig_t *config)
{
  ArmAbdEvalConfig_t local_config;

  if (config == 0)
  {
    ArmAbdEval_DefaultConfig(&local_config);
  }
  else
  {
    local_config = *config;
  }
  SanitizeArmAbdConfig(&local_config);

  memset(&arm_abd_ctx, 0, sizeof(arm_abd_ctx));
  arm_abd_ctx.config = local_config;
  arm_abd_ctx.state.target_reps = local_config.target_reps;
  arm_abd_ctx.state.active = 1U;
}

void ArmAbdEval_Stop(void)
{
  arm_abd_ctx.state.active = 0U;
  arm_abd_ctx.state.in_motion = 0U;
}

void ArmAbdEval_Update(const ArmAbdEvalSample_t *sample)
{
  ArmAbdEvalState_t *state = &arm_abd_ctx.state;

  if (sample == 0)
  {
    return;
  }

  state->just_completed_rep = 0U;
  state->just_finished = 0U;

  if ((state->active == 0U) || (state->finished != 0U))
  {
    return;
  }

  if (sample->valid == 0U)
  {
    state->current_angle_x100 = 0;
    state->in_motion = 0U;
    state->current_peak_x100 = 0;
    UpdateArmAbdScores();
    return;
  }

  state->current_angle_x100 = sample->abduction_angle_x100;

  if (state->in_motion == 0U)
  {
    if ((sample->abduction_angle_x100 >= arm_abd_ctx.config.enter_angle_x100) ||
        (AbsI32(sample->upper_yaw_gyro_x10) >= arm_abd_ctx.config.enter_gyro_x10))
    {
      state->in_motion = 1U;
      state->current_peak_x100 = sample->abduction_angle_x100;
    }
  }
  else
  {
    if (sample->abduction_angle_x100 > state->current_peak_x100)
    {
      state->current_peak_x100 = sample->abduction_angle_x100;
    }

    if ((sample->abduction_angle_x100 <= arm_abd_ctx.config.exit_angle_x100) &&
        (state->current_peak_x100 >= arm_abd_ctx.config.enter_angle_x100))
    {
      state->in_motion = 0U;
      state->reps++;
      state->just_completed_rep = 1U;

      if (state->current_peak_x100 > state->max_angle_x100)
      {
        state->max_angle_x100 = state->current_peak_x100;
      }

      arm_abd_ctx.peak_sum_x100 += (uint32_t)state->current_peak_x100;
      state->avg_peak_angle_x100 = (int32_t)(arm_abd_ctx.peak_sum_x100 / (uint32_t)state->reps);

      if (state->reps >= arm_abd_ctx.config.target_reps)
      {
        state->finished = 1U;
        state->active = 0U;
        state->just_finished = 1U;
      }
    }
  }

  UpdateArmAbdScores();
}

void ArmAbdEval_GetState(ArmAbdEvalState_t *state)
{
  if (state != 0)
  {
    *state = arm_abd_ctx.state;
  }
}

void ShoulderLiftEval_DefaultConfig(ShoulderLiftEvalConfig_t *config)
{
  if (config != 0)
  {
    config->target_reps = 5U;
    config->enter_angle_x100 = 1800;
    config->exit_angle_x100 = 1000;
    config->target_angle_x100 = 9000;
    config->enter_gyro_x10 = 80;
  }
}

void ShoulderLiftEval_Init(void)
{
  memset(&shoulder_lift_ctx, 0, sizeof(shoulder_lift_ctx));
}

void ShoulderLiftEval_Start(const ShoulderLiftEvalConfig_t *config)
{
  ShoulderLiftEvalConfig_t local_config;

  if (config == 0)
  {
    ShoulderLiftEval_DefaultConfig(&local_config);
  }
  else
  {
    local_config = *config;
  }
  SanitizeShoulderLiftConfig(&local_config);

  memset(&shoulder_lift_ctx, 0, sizeof(shoulder_lift_ctx));
  shoulder_lift_ctx.config = local_config;
  shoulder_lift_ctx.state.target_reps = local_config.target_reps;
  shoulder_lift_ctx.state.active = 1U;
}

void ShoulderLiftEval_Stop(void)
{
  shoulder_lift_ctx.state.active = 0U;
  shoulder_lift_ctx.state.in_motion = 0U;
}

void ShoulderLiftEval_Update(const ShoulderLiftEvalSample_t *sample)
{
  ShoulderLiftEvalState_t *state = &shoulder_lift_ctx.state;

  if (sample == 0)
  {
    return;
  }

  state->just_completed_rep = 0U;
  state->just_finished = 0U;

  if ((state->active == 0U) || (state->finished != 0U))
  {
    return;
  }

  if (sample->valid == 0U)
  {
    state->current_angle_x100 = 0;
    state->in_motion = 0U;
    state->current_peak_x100 = 0;
    UpdateShoulderLiftScores();
    return;
  }

  state->current_angle_x100 = sample->lift_angle_x100;

  if (state->in_motion == 0U)
  {
    if ((sample->lift_angle_x100 >= shoulder_lift_ctx.config.enter_angle_x100) ||
        (AbsI32(sample->upper_pitch_gyro_x10) >= shoulder_lift_ctx.config.enter_gyro_x10))
    {
      state->in_motion = 1U;
      state->current_peak_x100 = sample->lift_angle_x100;
    }
  }
  else
  {
    if (sample->lift_angle_x100 > state->current_peak_x100)
    {
      state->current_peak_x100 = sample->lift_angle_x100;
    }

    if ((sample->lift_angle_x100 <= shoulder_lift_ctx.config.exit_angle_x100) &&
        (state->current_peak_x100 >= shoulder_lift_ctx.config.enter_angle_x100))
    {
      state->in_motion = 0U;
      state->reps++;
      state->just_completed_rep = 1U;

      if (state->current_peak_x100 > state->max_angle_x100)
      {
        state->max_angle_x100 = state->current_peak_x100;
      }

      shoulder_lift_ctx.peak_sum_x100 += (uint32_t)state->current_peak_x100;
      state->avg_peak_angle_x100 =
          (int32_t)(shoulder_lift_ctx.peak_sum_x100 / (uint32_t)state->reps);

      if (state->reps >= shoulder_lift_ctx.config.target_reps)
      {
        state->finished = 1U;
        state->active = 0U;
        state->just_finished = 1U;
      }
    }
  }

  UpdateShoulderLiftScores();
}

void ShoulderLiftEval_GetState(ShoulderLiftEvalState_t *state)
{
  if (state != 0)
  {
    *state = shoulder_lift_ctx.state;
  }
}
