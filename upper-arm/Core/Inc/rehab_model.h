#ifndef __REHAB_MODEL_H__
#define __REHAB_MODEL_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  REHAB_MODEL_ARM_ABD = 0,
  REHAB_MODEL_ELBOW_FLEX = 1,
  REHAB_MODEL_FOREARM_ROT = 2,
  REHAB_MODEL_IDLE = 3,
  REHAB_MODEL_SHOULDER_LIFT = 4
} RehabModelAction_t;

typedef struct
{
  float elbow_deg;
  float hold;
  float fore_deg;
  float abd_deg;
  float shoulder_deg;
  float rel_deg;
  float elbow_vel_deg_s;
  float elbow_range_deg;
  float emg_active;
  float emg_rms_x10;
  float upper_gx_x10;
  float upper_gy_x10;
  float upper_gz_x10;
  float wrist_gx_x10;
  float wrist_gy_x10;
  float wrist_gz_x10;
  float abs_upper_gx_x10;
  float abs_upper_gy_x10;
  float abs_upper_gz_x10;
  float abs_wrist_gx_x10;
  float abs_wrist_gy_x10;
  float abs_wrist_gz_x10;
  float upper_gyro_norm_x10;
  float wrist_gyro_norm_x10;
  float rule_action_arm_abd;
  float rule_action_elbow_flex;
  float rule_action_forearm_rot;
  float rule_action_idle;
  float rule_action_shoulder_lift;
  float rule_latch_arm_abd;
  float rule_latch_elbow_flex;
  float rule_latch_forearm_rot;
  float rule_latch_idle;
  float rule_latch_shoulder_lift;
} RehabModelFeatures_t;

RehabModelAction_t RehabModel_Predict(const RehabModelFeatures_t *f);
const char *RehabModel_ActionName(RehabModelAction_t action);

#ifdef __cplusplus
}
#endif

#endif /* __REHAB_MODEL_H__ */
