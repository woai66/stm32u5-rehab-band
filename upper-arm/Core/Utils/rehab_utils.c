#include "rehab_utils.h"

#include <math.h>
#include <stddef.h>
/**
 * @brief 将浮点数值限制在指定的最小值和最大值范围内。
 *
 * @param value     需要被限制的输入值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 * @return float    如果 value 小于 min_value 则返回 min_value；
 *                  如果 value 大于 max_value 则返回 max_value；
 *                  否则返回 value 本身。
 */
float Rehab_ClampFloat(float value, float min_value, float max_value)
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
 * @brief 将32位整数值限制在指定的最小值和最大值范围内。
 *
 * @param value     需要被限制的输入值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 * @return int32_t  如果 value 小于 min_value 则返回 min_value；
 *                  如果 value 大于 max_value 则返回 max_value；
 *                  否则返回 value 本身。
 */
int32_t Rehab_ClampInt32(int32_t value, int32_t min_value, int32_t max_value)
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
 * @brief 将一个浮点数值从一个范围线性映射到另一个范围。
 *
 * @param value   需要映射的输入值。
 * @param in_min  输入范围的最小值。
 * @param in_max  输入范围的最大值。
 * @param out_min 输出范围的最小值。
 * @param out_max 输出范围的最大值。
 * @return float  映射后的值。如果输入范围无效（in_min == in_max），则返回 out_min。
 */
float Rehab_MapFloat(float value,
                     float in_min,
                     float in_max,
                     float out_min,
                     float out_max)
{
  float ratio;

  if (in_max == in_min)
  {
    return out_min;
  }

  ratio = (value - in_min) / (in_max - in_min);
  return out_min + (ratio * (out_max - out_min));
}
/**
 * @brief 计算浮点数值的绝对值。
 *
 * @param value 输入值。
 * @return float 输入值的绝对值。
 */
float Rehab_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}
/**
 * @brief 计算浮点数值的绝对值。
 *
 * @param value 输入值。
 * @return float 输入值的绝对值。
 */
float Rehab_DeadbandFloat(float value, float deadband)
{
  if (Rehab_AbsFloat(value) < deadband)
  {
    return 0.0f;
  }
  return value;
}
/**
 * @brief 初始化一阶低通滤波器结构体。
 *
 * @param filter        指向滤波器结构体的指针。
 * @param alpha         滤波系数，范围应在 [0.0, 1.0] 之间。
 *                      值越接近1，响应越快；值越接近0，平滑效果越强。
 * @param initial_value 滤波器的初始值。
 */
void Rehab_FirstOrderFilterInit(Rehab_FirstOrderFilter_t *filter,
                                float alpha,
                                float initial_value)
{
  if (filter == NULL)
  {
    return;
  }

  filter->alpha = Rehab_ClampFloat(alpha, 0.0f, 1.0f);
  filter->value = initial_value;
  filter->initialized = 1U;
}
/**
 * @brief 更新一阶低通滤波器并返回滤波后的值。
 *
 * @param filter 指向滤波器结构体的指针。
 * @param input  新的输入样本值。
 * @return float 滤波后的输出值。如果滤波器未初始化或指针为空，则返回当前输入或默认值。
 */

float Rehab_FirstOrderFilterUpdate(Rehab_FirstOrderFilter_t *filter,
                                   float input)
{
  if (filter == NULL)
  {
    return input;
  }

  if (filter->initialized == 0U)
  {
    filter->value = input;
    filter->initialized = 1U;
    return filter->value;
  }

  filter->value += filter->alpha * (input - filter->value);
  return filter->value;
}
/**
 * @brief 初始化滑动平均值滤波器结构体。
 *
 * @param avg    指向滑动平均结构体的指针。
 * @param buffer 用于存储历史数据的外部缓冲区指针。
 * @param length 缓冲区的长度（即滑动窗口的大小）。
 */
void Rehab_MovingAverageInit(Rehab_MovingAverage_t *avg,
                             float *buffer,
                             uint16_t length)
{
  if ((avg == NULL) || (buffer == NULL) || (length == 0U))
  {
    return;
  }

  avg->buffer = buffer;
  avg->length = length;
  avg->index = 0U;
  avg->count = 0U;
  avg->sum = 0.0f;

  for (uint16_t i = 0U; i < length; i++)
  {
    buffer[i] = 0.0f;
  }
}
/**
 * @brief 更新滑动平均值滤波器并返回当前的平均值。
 *
 * @param avg   指向滑动平均结构体的指针。
 * @param input 新的输入样本值。
 * @return float 当前窗口内的滑动平均值。如果参数无效，则返回输入值。
 */
float Rehab_MovingAverageUpdate(Rehab_MovingAverage_t *avg, float input)
{
  if ((avg == NULL) || (avg->buffer == NULL) || (avg->length == 0U))
  {
    return input;
  }

  if (avg->count < avg->length)
  {
    avg->count++;
  }
  else
  {
    avg->sum -= avg->buffer[avg->index];
  }

  avg->buffer[avg->index] = input;
  avg->sum += input;
  avg->index++;
  if (avg->index >= avg->length)
  {
    avg->index = 0U;
  }

  return avg->sum / (float)avg->count;
}
/**
 * @brief 计算浮点数组的均方根值 (RMS)。
 *
 * @param data   指向数据数组的指针。
 * @param length 数组的长度。
 * @return float 数据的均方根值。如果数据指针为空或长度为0，则返回0.0。
 */
float Rehab_RmsFloat(const float *data, uint16_t length)
{
  float sum_square = 0.0f;

  if ((data == NULL) || (length == 0U))
  {
    return 0.0f;
  }

  for (uint16_t i = 0U; i < length; i++)
  {
    sum_square += data[i] * data[i];
  }

  return sqrtf(sum_square / (float)length);
}
