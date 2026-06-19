#ifndef __IMU_PROCESSING_H__
#define __IMU_PROCESSING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lsm6dsr.h"
#include <stdint.h>

typedef struct
{
  float x;
  float y;
  float z;
} IMUProc_Vector3f_t;

typedef struct
{
  float w;
  float x;
  float y;
  float z;
} IMUProc_Quaternion_t;

typedef struct
{
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
} IMUProc_Euler_t;

typedef struct
{
  IMUProc_Vector3f_t acc_g;
  IMUProc_Vector3f_t gyro_dps;
} IMUProc_Sample_t;

typedef struct
{
  IMUProc_Vector3f_t gyro_bias_dps;
  IMUProc_Vector3f_t acc_static_g;
  uint32_t sample_count;
} IMUProc_Calib_t;

typedef struct
{
  float alpha;
  IMUProc_Sample_t value;
  uint8_t initialized;
} IMUProc_LowPass_t;

typedef struct
{
  IMUProc_Quaternion_t q;
  IMUProc_Euler_t euler;
  IMUProc_Vector3f_t integral_error;
  float kp;
  float ki;
  uint8_t initialized;
} IMUProc_Attitude_t;

typedef struct
{
  IMUProc_Calib_t calib;
  IMUProc_LowPass_t low_pass;
  IMUProc_Attitude_t attitude;
  IMUProc_Sample_t calibrated;
  IMUProc_Sample_t filtered;
} IMUProc_State_t;

typedef struct
{
  int32_t roll_x10;
  int32_t pitch_x10;
  int32_t yaw_x10;
} IMUProc_EulerX10_t;

void IMUProc_SampleFromLSM6DSR(const LSM6DSR_Data_t *raw, IMUProc_Sample_t *sample);

void IMUProc_CalibrationReset(IMUProc_Calib_t *calib);
void IMUProc_CalibrationAddSample(IMUProc_Calib_t *calib, const IMUProc_Sample_t *sample);
uint8_t IMUProc_CalibrationFinish(IMUProc_Calib_t *calib);
void IMUProc_ApplyCalibration(const IMUProc_Calib_t *calib,
                              const IMUProc_Sample_t *input,
                              IMUProc_Sample_t *output);

void IMUProc_LowPassInit(IMUProc_LowPass_t *filter, float alpha);
void IMUProc_LowPassUpdate(IMUProc_LowPass_t *filter,
                           const IMUProc_Sample_t *input,
                           IMUProc_Sample_t *output);

void IMUProc_AttitudeInit(IMUProc_Attitude_t *attitude);
void IMUProc_AttitudeSetGains(IMUProc_Attitude_t *attitude, float kp, float ki);
void IMUProc_AttitudeUpdate6Axis(IMUProc_Attitude_t *attitude,
                                 const IMUProc_Sample_t *sample,
                                 float dt_s);

void IMUProc_StateInit(IMUProc_State_t *state, float low_pass_alpha);
void IMUProc_PrepareRawForUpdate(LSM6DSR_Data_t *raw,
                                 const int32_t gyro_bias_raw[3],
                                 int32_t deadband_raw);
void IMUProc_StateUpdate(IMUProc_State_t *state,
                         const LSM6DSR_Data_t *raw,
                         float dt_s);
int32_t IMUProc_AngleTo360X10(float angle_deg);
int32_t IMUProc_ApplyAngleDeadbandX10(int32_t angle_x10, int32_t deadband_x10);
void IMUProc_EulerToX10(const IMUProc_Euler_t *euler,
                        int32_t deadband_x10,
                        IMUProc_EulerX10_t *out);

IMUProc_Quaternion_t IMUProc_QuaternionConjugate(IMUProc_Quaternion_t q);
IMUProc_Quaternion_t IMUProc_QuaternionMultiply(IMUProc_Quaternion_t a,
                                                IMUProc_Quaternion_t b);
IMUProc_Quaternion_t IMUProc_RelativeQuaternion(IMUProc_Quaternion_t reference,
                                                IMUProc_Quaternion_t target);
float IMUProc_RelativeAngleDeg(IMUProc_Quaternion_t reference,
                               IMUProc_Quaternion_t target);
float IMUProc_EstimateElbowFlexionDeg(const IMUProc_Attitude_t *upper_arm,
                                      const IMUProc_Attitude_t *forearm);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_PROCESSING_H__ */
