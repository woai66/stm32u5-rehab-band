#include "imu_processing.h"
#include <math.h>
#include <string.h>

#define IMUPROC_DEG_TO_RAD           (0.017453292519943295f)
#define IMUPROC_RAD_TO_DEG           (57.29577951308232f)
#define IMUPROC_DEFAULT_KP           (2.0f)
#define IMUPROC_DEFAULT_KI           (0.0f)
#define IMUPROC_MIN_DT_S             (0.0001f)
#define IMUPROC_MAX_DT_S             (0.1f)
#define IMUPROC_GYRO_DEADBAND_RAD    (0.00815f)

static float IMUProc_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}
/**
 * @brief ADC全传输完成回调函数。
 *
 * 当ADC DMA完成整个缓冲区的传输时调用，释放信号量以通知任务处理数据。
 *
 * @param hadc ADC句柄指针。
 */
static float IMUProc_VectorNorm(IMUProc_Vector3f_t v)
{
  return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}
/**
 * @brief 计算三维向量的欧几里得范数（模长）。
 *
 * @param v 输入三维向量。
 * @return float 向量的模长 sqrt(x^2 + y^2 + z^2)。
 */
static uint8_t IMUProc_NormalizeVector(IMUProc_Vector3f_t *v)
{
  float norm = IMUProc_VectorNorm(*v);

  if (norm < 0.000001f)
  {
    return 0U;
  }

  v->x /= norm;
  v->y /= norm;
  v->z /= norm;
  return 1U;
}
/**
 * @brief 对四元数进行归一化处理，确保其为单位四元数。
 *
 * @param q 指向待归一化四元数的指针。函数会直接修改该四元数。
 *          如果模长接近零，则重置为单位四元数 (1, 0, 0, 0)。
 */
static void IMUProc_NormalizeQuaternion(IMUProc_Quaternion_t *q)
{
  float norm = sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));

  if (norm < 0.000001f)
  {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
    return;
  }

  q->w /= norm;
  q->x /= norm;
  q->y /= norm;
  q->z /= norm;
}
/**
 * @brief 根据当前姿态四元数更新欧拉角（滚转、俯仰、偏航），单位为度。
 *
 * @param attitude 指向姿态结构体的指针，其中包含输入四元数和输出欧拉角字段。
 */
static void IMUProc_UpdateEuler(IMUProc_Attitude_t *attitude)
{
  IMUProc_Quaternion_t q = attitude->q;
  float sinr_cosp;
  float cosr_cosp;
  float sinp;
  float siny_cosp;
  float cosy_cosp;

  sinr_cosp = 2.0f * ((q.w * q.x) + (q.y * q.z));
  cosr_cosp = 1.0f - (2.0f * ((q.x * q.x) + (q.y * q.y)));
  attitude->euler.roll_deg = atan2f(sinr_cosp, cosr_cosp) * IMUPROC_RAD_TO_DEG;

  sinp = 2.0f * ((q.w * q.y) - (q.z * q.x));
  sinp = IMUProc_ClampFloat(sinp, -1.0f, 1.0f);
  attitude->euler.pitch_deg = asinf(sinp) * IMUPROC_RAD_TO_DEG;

  siny_cosp = 2.0f * ((q.w * q.z) + (q.x * q.y));
  cosy_cosp = 1.0f - (2.0f * ((q.y * q.y) + (q.z * q.z)));
  attitude->euler.yaw_deg = atan2f(siny_cosp, cosy_cosp) * IMUPROC_RAD_TO_DEG;
}
/**
 * @brief 从 LSM6DSR 原始数据结构中提取加速度和陀螺仪数据到样本结构体。
 *
 * @param raw    指向原始传感器数据的指针。
 * @param sample 指向输出样本结构体的指针。
 */
void IMUProc_SampleFromLSM6DSR(const LSM6DSR_Data_t *raw, IMUProc_Sample_t *sample)
{
  if ((raw == NULL) || (sample == NULL))
  {
    return;
  }

  sample->acc_g.x = raw->acc_g_x;
  sample->acc_g.y = raw->acc_g_y;
  sample->acc_g.z = raw->acc_g_z;
  sample->gyro_dps.x = raw->gyro_dps_x;
  sample->gyro_dps.y = raw->gyro_dps_y;
  sample->gyro_dps.z = raw->gyro_dps_z;
}
/**
 * @brief 重置校准结构体，将所有累积值和计数清零。
 *
 * @param calib 指向校准结构体的指针。
 */void IMUProc_CalibrationReset(IMUProc_Calib_t *calib)
{
  if (calib == NULL)
  {
    return;
  }

  memset(calib, 0, sizeof(*calib));
}
/**
 * @brief 添加一个采样点到校准累积器中。
 *        用于静止校准阶段，累加陀螺仪偏差和加速度计静态值。
 *
 * @param calib  指向校准结构体的指针。
 * @param sample 指向当前采样数据的指针。
 */
void IMUProc_CalibrationAddSample(IMUProc_Calib_t *calib, const IMUProc_Sample_t *sample)
{
  if ((calib == NULL) || (sample == NULL))
  {
    return;
  }

  calib->gyro_bias_dps.x += sample->gyro_dps.x;
  calib->gyro_bias_dps.y += sample->gyro_dps.y;
  calib->gyro_bias_dps.z += sample->gyro_dps.z;
  calib->acc_static_g.x += sample->acc_g.x;
  calib->acc_static_g.y += sample->acc_g.y;
  calib->acc_static_g.z += sample->acc_g.z;
  calib->sample_count++;
}
/**
 * @brief 完成校准计算，将累积值转换为平均值（偏差估计）。
 *
 * @param calib 指向校准结构体的指针。
 * @return uint8_t 如果校准成功完成（样本数大于0）返回 1U，否则返回 0U。
 */
uint8_t IMUProc_CalibrationFinish(IMUProc_Calib_t *calib)
{
  float inv_count;

  if ((calib == NULL) || (calib->sample_count == 0U))
  {
    return 0U;
  }

  inv_count = 1.0f / (float)calib->sample_count;
  calib->gyro_bias_dps.x *= inv_count;
  calib->gyro_bias_dps.y *= inv_count;
  calib->gyro_bias_dps.z *= inv_count;
  calib->acc_static_g.x *= inv_count;
  calib->acc_static_g.y *= inv_count;
  calib->acc_static_g.z *= inv_count;

  return 1U;
}
/**
 * @brief 应用校准偏差到原始采样数据。
 *        主要移除陀螺仪的零偏。
 *
 * @param calib  指向已计算好的校准结构体的指针。
 * @param input  指向输入原始采样数据的指针。
 * @param output 指向输出校准后采样数据的指针。
 */
void IMUProc_ApplyCalibration(const IMUProc_Calib_t *calib,
                              const IMUProc_Sample_t *input,
                              IMUProc_Sample_t *output)
{
  if ((calib == NULL) || (input == NULL) || (output == NULL))
  {
    return;
  }

  *output = *input;
  output->gyro_dps.x -= calib->gyro_bias_dps.x;
  output->gyro_dps.y -= calib->gyro_bias_dps.y;
  output->gyro_dps.z -= calib->gyro_bias_dps.z;
}
/**
 * @brief 初始化低通滤波器结构体。
 *
 * @param filter 指向低通滤波器结构体的指针。
 * @param alpha  滤波系数，范围应在 [0.0, 1.0] 之间。
 *               alpha 越接近 1，滤波效果越强（响应越慢）；
 *               alpha 越接近 0，响应越快（滤波效果越弱）。
 */
void IMUProc_LowPassInit(IMUProc_LowPass_t *filter, float alpha)
{
  if (filter == NULL)
  {
    return;
  }

  memset(filter, 0, sizeof(*filter));
  filter->alpha = IMUProc_ClampFloat(alpha, 0.0f, 1.0f);
}
/**
 * @brief 更新低通滤波器并获取滤波后的样本。
 *        使用指数移动平均算法：value_new = alpha * value_old + (1 - alpha) * input。
 *
 * @param filter 指向低通滤波器结构体的指针。
 * @param input  指向当前输入采样数据的指针。
 * @param output 指向输出滤波后采样数据的指针。
 */
void IMUProc_LowPassUpdate(IMUProc_LowPass_t *filter,
                           const IMUProc_Sample_t *input,
                           IMUProc_Sample_t *output)
{
  float a;
  float b;

  if ((filter == NULL) || (input == NULL) || (output == NULL))
  {
    return;
  }

  if (filter->initialized == 0U)
  {
    filter->value = *input;
    filter->initialized = 1U;
  }
  else
  {
    a = filter->alpha;
    b = 1.0f - a;

    filter->value.acc_g.x = (a * filter->value.acc_g.x) + (b * input->acc_g.x);
    filter->value.acc_g.y = (a * filter->value.acc_g.y) + (b * input->acc_g.y);
    filter->value.acc_g.z = (a * filter->value.acc_g.z) + (b * input->acc_g.z);
    filter->value.gyro_dps.x = (a * filter->value.gyro_dps.x) + (b * input->gyro_dps.x);
    filter->value.gyro_dps.y = (a * filter->value.gyro_dps.y) + (b * input->gyro_dps.y);
    filter->value.gyro_dps.z = (a * filter->value.gyro_dps.z) + (b * input->gyro_dps.z);
  }

  *output = filter->value;
}
/**
 * @brief 初始化姿态解算结构体。
 *        设置初始四元数为单位四元数，并配置默认 PID 增益。
 *
 * @param attitude 指向姿态结构体的指针。
 */
void IMUProc_AttitudeInit(IMUProc_Attitude_t *attitude)
{
  if (attitude == NULL)
  {
    return;
  }

  memset(attitude, 0, sizeof(*attitude));
  attitude->q.w = 1.0f;
  attitude->kp = IMUPROC_DEFAULT_KP;
  attitude->ki = IMUPROC_DEFAULT_KI;
  attitude->initialized = 1U;
}
/**
 * @brief 设置姿态解算器的比例 (Kp) 和积分 (Ki) 增益。
 *
 * @param attitude 指向姿态结构体的指针。
 * @param kp       比例增益，用于加速收敛。
 * @param ki       积分增益，用于消除陀螺仪漂移。
 */
void IMUProc_AttitudeSetGains(IMUProc_Attitude_t *attitude, float kp, float ki)
{
  if (attitude == NULL)
  {
    return;
  }

  attitude->kp = kp;
  attitude->ki = ki;
}
/**
 * @brief 基于六轴数据（加速度计+陀螺仪）更新姿态解算。
 *        使用互补滤波或 Mahony 滤波器算法融合数据。
 *
 * @param attitude 指向姿态结构体的指针，存储当前姿态和状态。
 * @param sample   指向经过校准和滤波后的采样数据的指针。
 * @param dt_s     时间步长（秒），用于积分计算。
 */
void IMUProc_AttitudeUpdate6Axis(IMUProc_Attitude_t *attitude,
                                 const IMUProc_Sample_t *sample,
                                 float dt_s)
{
  IMUProc_Vector3f_t acc;
  IMUProc_Vector3f_t error;
  IMUProc_Vector3f_t gravity;
  IMUProc_Quaternion_t q;
  float gx;
  float gy;
  float gz;
  float qw;
  float qx;
  float qy;
  float qz;

  if ((attitude == NULL) || (sample == NULL))
  {
    return;
  }

  if (attitude->initialized == 0U)
  {
    IMUProc_AttitudeInit(attitude);
  }

  dt_s = IMUProc_ClampFloat(dt_s, IMUPROC_MIN_DT_S, IMUPROC_MAX_DT_S);
  q = attitude->q;
  acc = sample->acc_g;

  gx = sample->gyro_dps.x * IMUPROC_DEG_TO_RAD;
  gy = sample->gyro_dps.y * IMUPROC_DEG_TO_RAD;
  gz = sample->gyro_dps.z * IMUPROC_DEG_TO_RAD;

  if (fabsf(gx) < IMUPROC_GYRO_DEADBAND_RAD)
  {
    gx = 0.0f;
  }
  if (fabsf(gy) < IMUPROC_GYRO_DEADBAND_RAD)
  {
    gy = 0.0f;
  }
  if (fabsf(gz) < IMUPROC_GYRO_DEADBAND_RAD)
  {
    gz = 0.0f;
  }

  if (IMUProc_NormalizeVector(&acc) != 0U)
  {
    gravity.x = 2.0f * ((q.x * q.z) - (q.w * q.y));
    gravity.y = 2.0f * ((q.w * q.x) + (q.y * q.z));
    gravity.z = (q.w * q.w) - (q.x * q.x) - (q.y * q.y) + (q.z * q.z);

    error.x = (acc.y * gravity.z) - (acc.z * gravity.y);
    error.y = (acc.z * gravity.x) - (acc.x * gravity.z);
    error.z = (acc.x * gravity.y) - (acc.y * gravity.x);

    if (attitude->ki > 0.0f)
    {
      attitude->integral_error.x += error.x * attitude->ki * dt_s;
      attitude->integral_error.y += error.y * attitude->ki * dt_s;
      attitude->integral_error.z += error.z * attitude->ki * dt_s;
    }
    else
    {
      attitude->integral_error.x = 0.0f;
      attitude->integral_error.y = 0.0f;
      attitude->integral_error.z = 0.0f;
    }

    gx += (attitude->kp * error.x) + attitude->integral_error.x;
    gy += (attitude->kp * error.y) + attitude->integral_error.y;
    gz += (attitude->kp * error.z) + attitude->integral_error.z;
  }

  qw = q.w;
  qx = q.x;
  qy = q.y;
  qz = q.z;

  q.w += 0.5f * ((-qx * gx) - (qy * gy) - (qz * gz)) * dt_s;
  q.x += 0.5f * (( qw * gx) + (qy * gz) - (qz * gy)) * dt_s;
  q.y += 0.5f * (( qw * gy) - (qx * gz) + (qz * gx)) * dt_s;
  q.z += 0.5f * (( qw * gz) + (qx * gy) - (qy * gx)) * dt_s;

  IMUProc_NormalizeQuaternion(&q);
  attitude->q = q;
  IMUProc_UpdateEuler(attitude);
}
/**
 * @brief 初始化完整的 IMU 处理状态结构体。
 *
 * @param state            指向状态结构体的指针。
 * @param low_pass_alpha   低通滤波器的 alpha 系数。
 */
void IMUProc_StateInit(IMUProc_State_t *state, float low_pass_alpha)
{
  if (state == NULL)
  {
    return;
  }

  memset(state, 0, sizeof(*state));
  IMUProc_CalibrationReset(&state->calib);
  IMUProc_LowPassInit(&state->low_pass, low_pass_alpha);
  IMUProc_AttitudeInit(&state->attitude);
}

void IMUProc_PrepareRawForUpdate(LSM6DSR_Data_t *raw,
                                 const int32_t gyro_bias_raw[3],
                                 int32_t deadband_raw)
{
  if ((raw == NULL) || (gyro_bias_raw == NULL))
  {
    return;
  }

  if ((raw->gyro_x > (gyro_bias_raw[0] - deadband_raw)) &&
      (raw->gyro_x < (gyro_bias_raw[0] + deadband_raw)))
  {
    raw->gyro_x = (int16_t)gyro_bias_raw[0];
  }
  if ((raw->gyro_y > (gyro_bias_raw[1] - deadband_raw)) &&
      (raw->gyro_y < (gyro_bias_raw[1] + deadband_raw)))
  {
    raw->gyro_y = (int16_t)gyro_bias_raw[1];
  }
  if ((raw->gyro_z > (gyro_bias_raw[2] - deadband_raw)) &&
      (raw->gyro_z < (gyro_bias_raw[2] + deadband_raw)))
  {
    raw->gyro_z = (int16_t)gyro_bias_raw[2];
  }
}

int32_t IMUProc_AngleTo360X10(float angle_deg)
{
  if (!((angle_deg >= -3600.0f) && (angle_deg <= 3600.0f)))
  {
    return 0;
  }

  while (angle_deg < 0.0f)
  {
    angle_deg += 360.0f;
  }
  while (angle_deg >= 360.0f)
  {
    angle_deg -= 360.0f;
  }

  return (int32_t)(angle_deg * 10.0f);
}

int32_t IMUProc_ApplyAngleDeadbandX10(int32_t angle_x10, int32_t deadband_x10)
{
  while (angle_x10 < 0)
  {
    angle_x10 += 3600;
  }
  while (angle_x10 >= 3600)
  {
    angle_x10 -= 3600;
  }

  if ((angle_x10 <= deadband_x10) || (angle_x10 >= (3600 - deadband_x10)))
  {
    return 0;
  }
  return angle_x10;
}

void IMUProc_EulerToX10(const IMUProc_Euler_t *euler,
                        int32_t deadband_x10,
                        IMUProc_EulerX10_t *out)
{
  if ((euler == NULL) || (out == NULL))
  {
    return;
  }

  out->roll_x10 = IMUProc_ApplyAngleDeadbandX10(IMUProc_AngleTo360X10(euler->roll_deg),
                                                deadband_x10);
  out->pitch_x10 = IMUProc_ApplyAngleDeadbandX10(IMUProc_AngleTo360X10(euler->pitch_deg),
                                                 deadband_x10);
  out->yaw_x10 = IMUProc_ApplyAngleDeadbandX10(IMUProc_AngleTo360X10(euler->yaw_deg),
                                               deadband_x10);
}
/**
 * @brief 主状态更新函数。依次执行数据提取、校准、滤波和姿态解算。
 *
 * @param state  指向 IMU 处理状态结构体的指针。
 * @param raw    指向原始 LSM6DSR 传感器数据的指针。
 * @param dt_s   距离上次更新的时间间隔（秒）。
 */
void IMUProc_StateUpdate(IMUProc_State_t *state,
                         const LSM6DSR_Data_t *raw,
                         float dt_s)
{
  IMUProc_Sample_t sample;

  if ((state == NULL) || (raw == NULL))
  {
    return;
  }

  IMUProc_SampleFromLSM6DSR(raw, &sample);
  IMUProc_ApplyCalibration(&state->calib, &sample, &state->calibrated);
  IMUProc_LowPassUpdate(&state->low_pass, &state->calibrated, &state->filtered);
  IMUProc_AttitudeUpdate6Axis(&state->attitude, &state->filtered, dt_s);
}
/**
 * @brief 计算四元数的共轭。
 *        对于单位四元数，共轭等于逆。
 *
 * @param q 输入四元数。
 * @return IMUProc_Quaternion_t 共轭四元数 (w, -x, -y, -z)。
 */
IMUProc_Quaternion_t IMUProc_QuaternionConjugate(IMUProc_Quaternion_t q)
{
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return q;
}

/**
 * @brief 计算两个四元数的乘积。
 *        结果表示旋转的组合。
 *
 * @param a 第一个四元数。
 * @param b 第二个四元数。
 * @return IMUProc_Quaternion_t 乘积四元数，并已归一化。
 */IMUProc_Quaternion_t IMUProc_QuaternionMultiply(IMUProc_Quaternion_t a,
                                                IMUProc_Quaternion_t b)
{
  IMUProc_Quaternion_t result;

  result.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  result.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  result.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
  result.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

  IMUProc_NormalizeQuaternion(&result);
  return result;
}
/**
 * @brief 计算从参考姿态到目标姿态的相对四元数。
 *        relative = conjugate(reference) * target
 *
 * @param reference 参考姿态四元数。
 * @param target    目标姿态四元数。
 * @return IMUProc_Quaternion_t 相对旋转四元数。
 */
IMUProc_Quaternion_t IMUProc_RelativeQuaternion(IMUProc_Quaternion_t reference,
                                                IMUProc_Quaternion_t target)
{
  return IMUProc_QuaternionMultiply(IMUProc_QuaternionConjugate(reference), target);
}
/**
 * @brief 计算两个姿态之间的相对夹角（度）。
 *
 * @param reference 参考姿态四元数。
 * @param target    目标姿态四元数。
 * @return float    两个姿态之间的最小夹角，范围为 [0, 180] 度。
 */
float IMUProc_RelativeAngleDeg(IMUProc_Quaternion_t reference,
                               IMUProc_Quaternion_t target)
{
  IMUProc_Quaternion_t relative = IMUProc_RelativeQuaternion(reference, target);
  float w = IMUProc_ClampFloat(fabsf(relative.w), -1.0f, 1.0f);

  return 2.0f * acosf(w) * IMUPROC_RAD_TO_DEG;
}
/**
 * @brief 估算肘部弯曲角度。
 *        基于上臂和前臂的姿态欧拉角中的俯仰角差值计算。
 *
 * @param upper_arm 指向代表上臂姿态的结构体指针。
 * @param forearm   指向代表前臂姿态的结构体指针。
 * @return float    估算的肘部弯曲角度，范围为 [0, 180] 度。
 */
float IMUProc_EstimateElbowFlexionDeg(const IMUProc_Attitude_t *upper_arm,
                                      const IMUProc_Attitude_t *forearm)
{
  float angle;

  if ((upper_arm == NULL) || (forearm == NULL))
  {
    return 0.0f;
  }

  angle = forearm->euler.pitch_deg - upper_arm->euler.pitch_deg;
  if (angle < 0.0f)
  {
    angle = -angle;
  }

  if (angle > 180.0f)
  {
    angle = 360.0f - angle;
  }

  return IMUProc_ClampFloat(angle, 0.0f, 180.0f);
}
