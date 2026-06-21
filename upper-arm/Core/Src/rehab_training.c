#include "rehab_training.h"

#include <string.h>

typedef struct
{
  RehabTrainingConfig_t config;
  RehabTrainingState_t state;
  uint32_t peak_sum_x100;
  uint16_t emg_active_reps;
  uint8_t current_rep_emg;
} RehabTrainingContext_t;

static RehabTrainingContext_t training_ctx;

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

static void UpdateScores(void)
{
  uint32_t completion;
  uint32_t amplitude;
  uint32_t participation;
  uint32_t total;

  completion = ScoreFromRatioU32(training_ctx.state.reps,
                                 training_ctx.config.target_reps);
  amplitude = ScoreFromRatioU32((uint32_t)training_ctx.state.max_angle_x100,
                                (uint32_t)training_ctx.config.target_angle_x100);
  participation = (training_ctx.state.reps == 0U) ?
      0U : ScoreFromRatioU32(training_ctx.emg_active_reps, training_ctx.state.reps);

  total = (completion * 40U + amplitude * 40U + participation * 20U) / 100U;

  training_ctx.state.completion_score = (uint8_t)completion;
  training_ctx.state.amplitude_score = (uint8_t)amplitude;
  training_ctx.state.participation_score = (uint8_t)participation;
  training_ctx.state.total_score = (uint8_t)total;
  training_ctx.state.emg_rate = (uint8_t)participation;
}

void RehabTraining_Init(void)
{
  memset(&training_ctx, 0, sizeof(training_ctx));
}

void RehabTraining_Start(const RehabTrainingConfig_t *config)
{
  RehabTrainingConfig_t local_config;

  if (config == 0)
  {
    local_config.action = REHAB_TRAIN_ACTION_ELBOW_FLEX;
    local_config.target_reps = 5U;
    local_config.enter_angle_x100 = 3000;
    local_config.exit_angle_x100 = 3000;
    local_config.target_angle_x100 = 15000;
  }
  else
  {
    local_config = *config;
  }

  if (local_config.target_reps == 0U)
  {
    local_config.target_reps = 5U;
  }
  if (local_config.target_angle_x100 <= 0)
  {
    local_config.target_angle_x100 = 15000;
  }
  if (local_config.exit_angle_x100 > local_config.enter_angle_x100)
  {
    local_config.exit_angle_x100 = local_config.enter_angle_x100 / 2;
  }

  memset(&training_ctx, 0, sizeof(training_ctx));
  training_ctx.config = local_config;
  training_ctx.state.action = local_config.action;
  training_ctx.state.target_reps = local_config.target_reps;
  training_ctx.state.active = 1U;
}

void RehabTraining_Stop(void)
{
  training_ctx.state.active = 0U;
  training_ctx.state.in_motion = 0U;
}

void RehabTraining_Update(const RehabTrainingSample_t *sample)
{
  RehabTrainingState_t *state = &training_ctx.state;

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

  if ((sample->valid == 0U) || (sample->action != training_ctx.config.action))
  {
    state->current_angle_x100 = (sample->valid != 0U) ? sample->angle_x100 : 0;
    state->emg_active = sample->emg_active;
    UpdateScores();
    return;
  }

  state->current_angle_x100 = sample->angle_x100;
  state->emg_active = sample->emg_active;

  if (state->in_motion == 0U)
  {
    if (sample->angle_x100 >= training_ctx.config.enter_angle_x100)
    {
      state->in_motion = 1U;
      state->current_peak_x100 = sample->angle_x100;
      training_ctx.current_rep_emg = sample->emg_active;
    }
  }
  else
  {
    if (sample->angle_x100 > state->current_peak_x100)
    {
      state->current_peak_x100 = sample->angle_x100;
    }
    if (sample->emg_active != 0U)
    {
      training_ctx.current_rep_emg = 1U;
    }

    if (sample->angle_x100 <= training_ctx.config.exit_angle_x100)
    {
      state->in_motion = 0U;
      state->reps++;
      state->just_completed_rep = 1U;

      if (state->current_peak_x100 > state->max_angle_x100)
      {
        state->max_angle_x100 = state->current_peak_x100;
      }

      training_ctx.peak_sum_x100 += (uint32_t)state->current_peak_x100;
      state->avg_peak_angle_x100 =
          (int32_t)(training_ctx.peak_sum_x100 / (uint32_t)state->reps);

      if (training_ctx.current_rep_emg != 0U)
      {
        training_ctx.emg_active_reps++;
      }
      training_ctx.current_rep_emg = 0U;

      if (state->reps >= training_ctx.config.target_reps)
      {
        state->finished = 1U;
        state->active = 0U;
        state->just_finished = 1U;
      }
    }
  }

  UpdateScores();
}

void RehabTraining_GetState(RehabTrainingState_t *state)
{
  if (state != 0)
  {
    *state = training_ctx.state;
  }
}

const char *RehabTraining_ActionName(RehabTrainingAction_t action)
{
  switch (action)
  {
    case REHAB_TRAIN_ACTION_ELBOW_FLEX:
      return "ELBOW_FLEX";
    case REHAB_TRAIN_ACTION_ARM_ABDUCTION:
      return "ARM_ABD";
    case REHAB_TRAIN_ACTION_FOREARM_ROTATION:
      return "FOREARM_ROT";
    case REHAB_TRAIN_ACTION_SHOULDER_LIFT:
      return "SHOULDER_LIFT";
    default:
      return "UNKNOWN";
  }
}
