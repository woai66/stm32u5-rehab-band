/* ==================== Include ==================== */
#include "emg_sensor.h"

#include "adc.h"
#include "app_freertos.h"
#include "rehab_utils.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* ==================== 宏定义 ==================== */
#define EMG_SENSOR_ADC_MAX             (4095.0f)
#define EMG_SENSOR_VREF_MV             (3300.0f)
/* 500Hz 下采集 2 秒静息数据，用于建立个体化基线和噪声水平。 */
#define EMG_SENSOR_BASELINE_SAMPLES    (EMG_SENSOR_SAMPLE_RATE_HZ * 2U)
#define EMG_SENSOR_WARMUP_SAMPLES      (EMG_SENSOR_SAMPLE_RATE_HZ * 3U)
#define EMG_SENSOR_ENVELOPE_ALPHA      (0.15f)
/* 64 点约等于 128ms 窗口，用于估计肌电短时 RMS。 */
#define EMG_SENSOR_RMS_WINDOW_LEN      (64U)
/* 噪声阈值倍数，避免静息抖动被误判为主动发力。 */
#define EMG_SENSOR_NOISE_MULTIPLIER    (4.0f)
#define EMG_SENSOR_BASE_DEBUG          (0U)

/* ==================== 全局静态变量 ==================== */
/* ADC DMA 环形缓冲区，半满/满中断分别处理前后 32 点。 */
static uint16_t emg_adc_buffer[EMG_SENSOR_BUFFER_LEN];
/* RMS 不直接保存原始 ADC，而是保存去基线后的幅值平方滑动均值。 */
static float emg_rms_window[EMG_SENSOR_RMS_WINDOW_LEN];
static EmgSensor_State_t emg_state;
/* baseline_square_sum 用 E[x^2] - E[x]^2 估计静息噪声方差。 */
static uint32_t emg_baseline_sum;
static uint64_t emg_baseline_square_sum;
static uint32_t emg_baseline_count;
static uint32_t emg_warmup_count;
static float emg_active_delta = EMG_SENSOR_DEFAULT_ACTIVE_DELTA;
static float emg_full_scale_delta = EMG_SENSOR_DEFAULT_FULL_SCALE_DELTA;
static Rehab_FirstOrderFilter_t emg_envelope_filter;
static Rehab_MovingAverage_t emg_rms_average;

/**
 * @brief 完成静息标定，计算 baseline、noise_level 并初始化后续滤波器。
 *
 * baseline 用于去除传感器直流偏置；noise_level 用于估计静息噪声强度。
 * 若噪声推导出的阈值高于默认阈值，则自动抬高激活阈值，减少误触发。
 */
static void EmgSensor_FinishBaseline(void)
{
  float mean;
  float mean_square;
  float variance;
  float threshold_from_noise;

  if (emg_baseline_count == 0U)
  {
    return;
  }

  mean = (float)emg_baseline_sum / (float)emg_baseline_count;
  mean_square = (float)emg_baseline_square_sum / (float)emg_baseline_count;
  variance = mean_square - (mean * mean);
  /* 浮点舍入可能让理论上非负的方差变成极小负数。 */
  if (variance < 0.0f)
  {
    variance = 0.0f;
  }

  emg_state.baseline = mean;
  emg_state.noise_level = sqrtf(variance);
  emg_state.rectified = 0.0f;
  emg_state.envelope = 0.0f;
  emg_state.rms = 0.0f;
  emg_state.active = 0U;
  emg_state.strength = 0U;
  emg_state.participation = 0U;
  emg_state.calibrated = 1U;

  if (EMG_SENSOR_BASE_DEBUG != 0U)
  {
    printf("EMG_BASE_DBG count=%lu sum=%lu mean_x10=%ld noise_x10=%ld state_size=%u\r\n",
           (unsigned long)emg_baseline_count,
           (unsigned long)emg_baseline_sum,
           (long)(mean * 10.0f),
           (long)(emg_state.noise_level * 10.0f),
           (unsigned int)sizeof(EmgSensor_State_t));
  }

  threshold_from_noise = emg_state.noise_level * EMG_SENSOR_NOISE_MULTIPLIER;
  if (threshold_from_noise > emg_active_delta)
  {
    emg_active_delta = threshold_from_noise;
  }

  /* 标定完成后再启动包络滤波和 RMS 滑动窗口，避免静息样本污染有效特征。 */
  Rehab_FirstOrderFilterInit(&emg_envelope_filter, EMG_SENSOR_ENVELOPE_ALPHA, 0.0f);
  Rehab_MovingAverageInit(&emg_rms_average, emg_rms_window, EMG_SENSOR_RMS_WINDOW_LEN);
}

/**
 * @brief 处理单路EMG原始采样数据，更新传感器状态
 * 该函数执行以下操作：
 * 1. 将原始ADC值转换为电压值
 * 2. 若未校准，累积静息样本计算基线均值和噪声方差
 * 3. 若已校准，计算信号相对基线的正向幅值，提取包络线和 RMS
 *    并根据阈值判断激活状态、强度百分比和参与度
 * @param raw 原始ADC采样值
 */
static void EmgSensor_ProcessSample(uint16_t raw)
{
  float signal;
  float strength;
  float rms_mean_square;

  emg_state.raw = raw;
  emg_state.voltage_mv = ((float)raw * EMG_SENSOR_VREF_MV) / EMG_SENSOR_ADC_MAX;

  if (emg_state.calibrated == 0U)
  {
    if (emg_warmup_count < EMG_SENSOR_WARMUP_SAMPLES)
    {
      emg_warmup_count++;
      return;
    }

    /* 标定阶段要求肌肉保持放松，样本只用于建立静息基线和噪声水平。 */
    emg_baseline_sum += raw;
    emg_baseline_square_sum += ((uint64_t)raw * (uint64_t)raw);
    emg_baseline_count++;

    if (emg_baseline_count >= EMG_SENSOR_BASELINE_SAMPLES)
    {
      EmgSensor_FinishBaseline();
    }
    return;
  }

  /* Some EMG front-ends drop voltage during contraction, so use magnitude. */
  signal = fabsf((float)raw - emg_state.baseline);

  emg_state.rectified = signal;
  /* envelope 适合做实时激活判断，RMS 更适合后续做训练窗口特征。 */
  emg_state.envelope = Rehab_FirstOrderFilterUpdate(&emg_envelope_filter, signal);
  rms_mean_square = Rehab_MovingAverageUpdate(&emg_rms_average, signal * signal);
  emg_state.rms = sqrtf(rms_mean_square);
  emg_state.active = (emg_state.envelope >= emg_active_delta) ? 1U : 0U;

  strength = (emg_state.envelope * 100.0f) / emg_full_scale_delta;
  strength = Rehab_ClampFloat(strength, 0.0f, 100.0f);
  emg_state.strength = (uint8_t)(strength + 0.5f);
  /* 当前版本先用强度近似参与度；后续应结合动作有效时间计算参与率。 */
  emg_state.participation = emg_state.strength;
}

/**
 * @brief 处理ADC缓冲区指定区间的数据样本
 * @param start 起始索引
 * @param count 需要处理的样本数量
 */
static void EmgSensor_ProcessRange(uint32_t start, uint32_t count)
{
  uint32_t end = start + count;

  /* 防止错误的 DMA 区间参数越界访问采样缓冲区。 */
  if (end > EMG_SENSOR_BUFFER_LEN)
  {
    return;
  }

  for (uint32_t i = start; i < end; i++)
  {
    EmgSensor_ProcessSample(emg_adc_buffer[i]);
  }
}

/**
 * @brief 启动EMG传感器数据采集
 * 初始化状态变量与缓冲区，启动定时器触发ADC，并开启ADC DMA传输
 * @return HAL_StatusTypeDef HAL_OK表示成功，HAL_ERROR表示失败
 */
HAL_StatusTypeDef EmgSensor_Start(void)
{
  /* 每次启动采集都重新进入静息标定流程。 */
  memset(&emg_state, 0, sizeof(emg_state));
  memset(emg_adc_buffer, 0, sizeof(emg_adc_buffer));
  memset(emg_rms_window, 0, sizeof(emg_rms_window));
  emg_baseline_sum = 0U;
  emg_baseline_square_sum = 0U;
  emg_baseline_count = 0U;
  emg_warmup_count = 0U;

  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_ADC_Start_DMA(&hadc1,
                           (uint32_t *)emg_adc_buffer,
                           EMG_SENSOR_BUFFER_LEN);
}

/**
 * @brief 处理ADC DMA半传输完成中断对应的数据段
 * 处理缓冲区前半段数据
 */
void EmgSensor_ProcessHalfBuffer(void)
{
  EmgSensor_ProcessRange(0U, EMG_SENSOR_BUFFER_LEN / 2U);
}

/**
 * @brief 处理ADC DMA全传输完成中断对应的数据段
 * 处理缓冲区后半段数据
 */
void EmgSensor_ProcessFullBuffer(void)
{
  EmgSensor_ProcessRange(EMG_SENSOR_BUFFER_LEN / 2U, EMG_SENSOR_BUFFER_LEN / 2U);
}

/**
 * @brief 获取当前EMG传感器状态副本
 * 使用临界段保护，保证状态读取一致性
 * @param state 指向存储状态副本的EmgSensor_State_t结构体指针
 */
void EmgSensor_GetState(EmgSensor_State_t *state)
{
  if (state == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  *state = emg_state;
  taskEXIT_CRITICAL();
}

void EmgSensor_GetFeatures(EmgSensor_Features_t *features)
{
  float drop;

  if (features == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  features->raw = emg_state.raw;
  features->baseline_x10 = (int32_t)(emg_state.baseline * 10.0f);
  drop = emg_state.baseline - (float)emg_state.raw;
  if (drop < 0.0f)
  {
    drop = 0.0f;
  }
  features->drop_x10 = (int32_t)(drop * 10.0f);
  features->rectified_x10 = (int32_t)(emg_state.rectified * 10.0f);
  features->envelope_x10 = (int32_t)(emg_state.envelope * 10.0f);
  features->rms_x10 = (int32_t)(emg_state.rms * 10.0f);
  features->active = emg_state.active;
  features->strength = emg_state.strength;
  features->calibrated = emg_state.calibrated;
  taskEXIT_CRITICAL();
}

/**
 * @brief 设置EMG信号处理阈值参数
 * @param active_delta 激活阈值，必须大于0
 * @param full_scale_delta 满量程阈值，必须大于active_delta
 */
void EmgSensor_SetThreshold(float active_delta, float full_scale_delta)
{
  /* 保留手动阈值接口，方便现场根据贴片位置和个体差异临时调参。 */
  if (active_delta > 0.0f)
  {
    emg_active_delta = active_delta;
  }
  if (full_scale_delta > emg_active_delta)
  {
    emg_full_scale_delta = full_scale_delta;
  }
}

/**
 * @brief 格式化新版 EMG 调试行。
 *
 * 当前函数只封装输出，不在 StartEmgTask 中调用，避免影响现有测试日志格式。
 * 后续可由 DebugTask、WirelessTask 或数据记录任务按需调用。
 */
int EmgSensor_FormatDebugLine(char *buffer, size_t buffer_len)
{
  EmgSensor_State_t state;

  if ((buffer == NULL) || (buffer_len == 0U))
  {
    return 0;
  }

  EmgSensor_GetState(&state);

  return snprintf(buffer,
                  buffer_len,
                  "EMG2 raw=%u mv=%lu base_x10=%ld noise_x10=%ld rect_x10=%ld env_x10=%ld rms_x10=%ld active=%u strength=%u participation=%u cal=%u\r\n",
                  (unsigned int)state.raw,
                  (unsigned long)(state.voltage_mv + 0.5f),
                  (long)(state.baseline * 10.0f),
                  (long)(state.noise_level * 10.0f),
                  (long)(state.rectified * 10.0f),
                  (long)(state.envelope * 10.0f),
                  (long)(state.rms * 10.0f),
                  (unsigned int)state.active,
                  (unsigned int)state.strength,
                  (unsigned int)state.participation,
                  (unsigned int)state.calibrated);
}

/**
 * @brief ADC半传输完成回调函数
 * ADC DMA完成缓冲区前半段传输时触发，释放信号量通知任务处理数据
 * @param hadc ADC句柄指针
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (adcHalfCpltSemHandle != NULL))
  {
    (void)osSemaphoreRelease(adcHalfCpltSemHandle);
  }
}

/**
 * @brief ADC全传输完成回调函数
 * ADC DMA完成整个缓冲区传输时触发，释放信号量通知任务处理数据
 * @param hadc ADC句柄指针
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (adcCpltSemHandle != NULL))
  {
    (void)osSemaphoreRelease(adcCpltSemHandle);
  }
}
