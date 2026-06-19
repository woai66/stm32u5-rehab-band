/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_touch_test.h"
#include "lsm6dsr.h"
#include "emg_sensor.h"
#include "emg_rf_model.h"
#include "imu_processing.h"
#include "wireless_link.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_ATTITUDE_KP    (0.004f)
#define IMU_ATTITUDE_KI    (0.00005f)
#define IMU_ANGLE_DEADBAND_X10  (5)
#define IMU_GYRO_DEADBAND_RAW  (2)
#define DATA_STREAM_PERIOD_MS  (20U)
#define EMG_UART4_RECORD_MODE  (1U)
/* 采样任务里的陀螺调试打印默认关闭：fputc 逐字节阻塞写 UART4，会拖慢 200Hz 采样周期 */
#define SENSOR_TASK_GYRO_DEBUG (0U)
/* 25% relative EMG amplitude is treated as full effort for the simple force estimate. */
#define EMG_FORCE_FULL_SCALE_PERCENT_X10  (250U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Upper-arm node local IMU cache. */
static LSM6DSR_Data_t debug_imu_raw;
static IMUProc_Euler_t debug_imu_euler;
static uint8_t debug_imu_valid;

/* USER CODE END Variables */

/* USER CODE BEGIN 0 */
static uint8_t Emg_EstimateForcePercent(const EmgSensor_Features_t *features)
{
  uint32_t drop_percent_x10;
  uint32_t env_percent_x10;
  uint32_t rms_percent_x10;
  uint32_t score_percent_x10;
  uint32_t force;

  if ((features == NULL) ||
      (features->calibrated == 0U) ||
      (features->baseline_x10 <= 0) ||
      (features->drop_x10 <= 0))
  {
    return 0U;
  }

  drop_percent_x10 = ((uint32_t)features->drop_x10 * 1000U) / (uint32_t)features->baseline_x10;
  env_percent_x10 = ((uint32_t)features->envelope_x10 * 1000U) / (uint32_t)features->baseline_x10;
  rms_percent_x10 = ((uint32_t)features->rms_x10 * 1000U) / (uint32_t)features->baseline_x10;

  score_percent_x10 = ((drop_percent_x10 * 7U) +
                       (env_percent_x10 * 2U) +
                       rms_percent_x10) / 10U;
  force = (score_percent_x10 * 100U) / EMG_FORCE_FULL_SCALE_PERCENT_X10;
  if (force > 100U)
  {
    force = 100U;
  }

  return (uint8_t)force;
}

static void DebugUart_WriteBuffer(const char *buffer, int length)
{
  if ((buffer == NULL) || (length <= 0))
  {
    return;
  }

  if (length > 255)
  {
    length = 255;
  }

  (void)HAL_UART_Transmit(&huart4, (uint8_t *)buffer, (uint16_t)length, 100U);
}

/* USER CODE END 0 */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for AlgoTask */
osThreadId_t AlgoTaskHandle;
const osThreadAttr_t AlgoTask_attributes = {
  .name = "AlgoTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for EmgTask */
osThreadId_t EmgTaskHandle;
const osThreadAttr_t EmgTask_attributes = {
  .name = "EmgTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for WirelessTask */
osThreadId_t WirelessTaskHandle;
const osThreadAttr_t WirelessTask_attributes = {
  .name = "WirelessTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for DataStreamTask */
osThreadId_t DataStreamTaskHandle;
const osThreadAttr_t DataStreamTask_attributes = {
  .name = "DataStreamTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for UiTask */
osThreadId_t UiTaskHandle;
const osThreadAttr_t UiTask_attributes = {
  .name = "UiTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 384 * 4
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 2048 * 4
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};
/* Definitions for uart1Mutex */
osMutexId_t uart1MutexHandle;
const osMutexAttr_t uart1Mutex_attributes = {
  .name = "uart1Mutex"
};
/* Definitions for uart2Mutex */
osMutexId_t uart2MutexHandle;
const osMutexAttr_t uart2Mutex_attributes = {
  .name = "uart2Mutex"
};
/* Definitions for spi3LcdMutex */
osMutexId_t spi3LcdMutexHandle;
const osMutexAttr_t spi3LcdMutex_attributes = {
  .name = "spi3LcdMutex"
};
/* Definitions for rehabStateMutex */
osMutexId_t rehabStateMutexHandle;
const osMutexAttr_t rehabStateMutex_attributes = {
  .name = "rehabStateMutex"
};
/* Definitions for imuFrameQueue */
osMessageQueueId_t imuFrameQueueHandle;
const osMessageQueueAttr_t imuFrameQueue_attributes = {
  .name = "imuFrameQueue"
};
/* Definitions for rehabStateQueue */
osMessageQueueId_t rehabStateQueueHandle;
const osMessageQueueAttr_t rehabStateQueue_attributes = {
  .name = "rehabStateQueue"
};
/* Definitions for debugLogQueue */
osMessageQueueId_t debugLogQueueHandle;
const osMessageQueueAttr_t debugLogQueue_attributes = {
  .name = "debugLogQueue"
};
/* Definitions for imuDmaDoneSem */
osSemaphoreId_t imuDmaDoneSemHandle;
const osSemaphoreAttr_t imuDmaDoneSem_attributes = {
  .name = "imuDmaDoneSem"
};
/* Definitions for adcHalfCpltSem */
osSemaphoreId_t adcHalfCpltSemHandle;
const osSemaphoreAttr_t adcHalfCpltSem_attributes = {
  .name = "adcHalfCpltSem"
};
/* Definitions for adcCpltSem */
osSemaphoreId_t adcCpltSemHandle;
const osSemaphoreAttr_t adcCpltSem_attributes = {
  .name = "adcCpltSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* creation of uart1Mutex */
  uart1MutexHandle = osMutexNew(&uart1Mutex_attributes);

  /* creation of uart2Mutex */
  uart2MutexHandle = osMutexNew(&uart2Mutex_attributes);

  /* creation of spi3LcdMutex */
  spi3LcdMutexHandle = osMutexNew(&spi3LcdMutex_attributes);

  /* creation of rehabStateMutex */
  rehabStateMutexHandle = osMutexNew(&rehabStateMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of imuDmaDoneSem */
  imuDmaDoneSemHandle = osSemaphoreNew(1, 0, &imuDmaDoneSem_attributes);

  /* creation of adcHalfCpltSem */
  adcHalfCpltSemHandle = osSemaphoreNew(1, 0, &adcHalfCpltSem_attributes);

  /* creation of adcCpltSem */
  adcCpltSemHandle = osSemaphoreNew(1, 0, &adcCpltSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of imuFrameQueue */
  imuFrameQueueHandle = osMessageQueueNew (4, 48, &imuFrameQueue_attributes);
  /* creation of rehabStateQueue */
  rehabStateQueueHandle = osMessageQueueNew (2, 32, &rehabStateQueue_attributes);
  /* creation of debugLogQueue */
  debugLogQueueHandle = osMessageQueueNew (8, 64, &debugLogQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of AlgoTask */
  AlgoTaskHandle = osThreadNew(StartAlgoTask, NULL, &AlgoTask_attributes);

  /* creation of EmgTask */
  EmgTaskHandle = osThreadNew(StartEmgTask, NULL, &EmgTask_attributes);

  /* creation of WirelessTask */
  WirelessTaskHandle = osThreadNew(StartWirelessTask, NULL, &WirelessTask_attributes);

  /* creation of DataStreamTask */
  DataStreamTaskHandle = osThreadNew(StartDataStreamTask, NULL, &DataStreamTask_attributes);

  /* creation of UiTask */
  UiTaskHandle = osThreadNew(StartUiTask, NULL, &UiTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN SensorTask */
  /* This task samples and solves the upper-arm IMU only. */
  LSM6DSR_Data_t imu_data;
  IMUProc_State_t imu_state;
  int32_t gyro_bias_raw[3] = {0};// 闄€铻哄師濮嬮浂鍋忓潎鍊?
  int32_t gyro_bias_sum[3] = {0};// 鏍″噯绱姞鍜?
  uint32_t last_error_tick = 0U;
  uint32_t last_gyro_print_tick = 0U;
  uint16_t calib_count = 0U;

  IMUProc_StateInit(&imu_state, 0.2f);
  IMUProc_AttitudeSetGains(&imu_state.attitude, IMU_ATTITUDE_KP, IMU_ATTITUDE_KI);
//  DebugUart_WriteBuffer("SensorTask UART4 debug start\r\n", sizeof("SensorTask UART4 debug start\r\n") - 1);

  while (LSM6DSR_Init() != HAL_OK)
  {
    if ((osKernelGetTickCount() - last_error_tick) >= 500U)
    {
      last_error_tick = osKernelGetTickCount();
      printf("LSM start failed, WHO_AM_I=0x%02X\r\n", LSM6DSR_ReadWhoAmI());
    }
    osDelay(100);
  }

  printf("Upper IMU start OK, keep still for calibration\r\n");

  // 棰勯噰闆嗛潤缃ǔ瀹?
  for (uint16_t i = 0U; i < 100U; i++)
  {
    (void)LSM6DSR_ReadRaw(&imu_data);
    osDelay(5);
  }

  IMUProc_CalibrationReset(&imu_state.calib);
  // 闈欐€侀浂鍋忔牎鍑?
  while (calib_count < 600U)
  {
    if (LSM6DSR_ReadRaw(&imu_data) == HAL_OK)
    {
      gyro_bias_sum[0] += imu_data.gyro_x;
      gyro_bias_sum[1] += imu_data.gyro_y;
      gyro_bias_sum[2] += imu_data.gyro_z;
      calib_count++;
    }
    osDelay(5);
  }

  gyro_bias_raw[0] = gyro_bias_sum[0] / (int32_t)calib_count;
  gyro_bias_raw[1] = gyro_bias_sum[1] / (int32_t)calib_count;
  gyro_bias_raw[2] = gyro_bias_sum[2] / (int32_t)calib_count;
  imu_state.calib.gyro_bias_dps.x = (float)gyro_bias_raw[0] * 0.070f;
  imu_state.calib.gyro_bias_dps.y = (float)gyro_bias_raw[1] * 0.070f;
  imu_state.calib.gyro_bias_dps.z = (float)gyro_bias_raw[2] * 0.070f;
  IMUProc_AttitudeInit(&imu_state.attitude);
  IMUProc_AttitudeSetGains(&imu_state.attitude, IMU_ATTITUDE_KP, IMU_ATTITUDE_KI);

  printf("Upper IMU calib OK, gyro_bias_raw=%d,%d,%d sample_raw=%d,%d,%d\r\n",
         (int)gyro_bias_raw[0],
         (int)gyro_bias_raw[1],
         (int)gyro_bias_raw[2],
         (int)imu_data.gyro_x,
         (int)imu_data.gyro_y,
         (int)imu_data.gyro_z);

  /* Infinite loop */
  for(;;)
  {
    if (LSM6DSR_ReadRaw(&imu_data) == HAL_OK)
    {
      IMUProc_PrepareRawForUpdate(&imu_data, gyro_bias_raw, IMU_GYRO_DEADBAND_RAW);// 鍑忓幓闆跺亸+姝诲尯杩囨护寰皬鎶栧姩
      IMUProc_StateUpdate(&imu_state, &imu_data, 0.005f);// 濮挎€佽瀺鍚堟洿鏂?

      taskENTER_CRITICAL();
      /* Publish upper-arm IMU data for debug, display, and later relative-pose calculation. */
      debug_imu_raw = imu_data;
      debug_imu_euler = imu_state.attitude.euler;
      debug_imu_valid = 1U;
      taskEXIT_CRITICAL();

#if (SENSOR_TASK_GYRO_DEBUG != 0U)
      if ((osKernelGetTickCount() - last_gyro_print_tick) >= 200U)
      {
        last_gyro_print_tick = osKernelGetTickCount();
        printf("GYRO raw=%d,%d,%d dps_x10=%ld,%ld,%ld\r\n",
               (int)imu_data.gyro_x,
               (int)imu_data.gyro_y,
               (int)imu_data.gyro_z,
               (long)(imu_data.gyro_dps_x * 10.0f),
               (long)(imu_data.gyro_dps_y * 10.0f),
               (long)(imu_data.gyro_dps_z * 10.0f));
      }
#else
      (void)last_gyro_print_tick;
#endif

//      if ((osKernelGetTickCount() - last_gyro_print_tick) >= 100U)
//      {
//        last_gyro_print_tick = osKernelGetTickCount();
//        debug_len = snprintf(debug_line,
//                             sizeof(debug_line),
//                             "IMU gyro_dps=%.2f,%.2f,%.2f acc_g=%.3f,%.3f,%.3f angle=%.2f,%.2f,%.2f\r\n",
//                             (double)imu_data.gyro_dps_x,
//                             (double)imu_data.gyro_dps_y,
//                             (double)imu_data.gyro_dps_z,
//                             (double)imu_data.acc_g_x,
//                             (double)imu_data.acc_g_y,
//                             (double)imu_data.acc_g_z,
//                             (double)imu_state.attitude.euler.roll_deg,
//                             (double)imu_state.attitude.euler.pitch_deg,
//                             (double)imu_state.attitude.euler.yaw_deg);
//        DebugUart_WriteBuffer(debug_line, debug_len);
//      }
    }
    else
    {
      if ((osKernelGetTickCount() - last_error_tick) >= 500U)
      {
        last_error_tick = osKernelGetTickCount();
        printf("LSM read failed\r\n");
      }
    }
    osDelay(5);
  }
  /* USER CODE END SensorTask */
}

/* USER CODE BEGIN Header_StartAlgoTask */
/**
* @brief Function implementing the AlgoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlgoTask */
void StartAlgoTask(void *argument)
{
  /* USER CODE BEGIN AlgoTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AlgoTask */
}

/* USER CODE BEGIN Header_StartEmgTask */
/**
* @brief Function implementing the EmgTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEmgTask */
// DMA 鍙岀紦鍐?ADC 鑲岀數閲囬泦浠诲姟锛岄厤鍚?ADC DMA 鍗婂畬鎴?/ 鍏ㄥ畬鎴愪俊鍙烽噺鍋氬垎鍧楁暟鎹鐞?
void StartEmgTask(void *argument)
{
  /* USER CODE BEGIN EmgTask */
  EmgSensor_Features_t emg_features;
  EmgModelInput_t model_input;
  EmgAction_t emg_action;
  uint16_t confidence_x1000 = 0U;
  uint8_t flex_level = 0U;
  uint8_t force_percent = 0U;
  uint32_t last_print_tick = 0U;

  // EMG 纭欢鍚姩鍒濆鍖?
  if (EmgSensor_Start() != HAL_OK)
  {
    if (EMG_UART4_RECORD_MODE == 0U)
    {
      osMutexAcquire(uart2MutexHandle, osWaitForever);
//      printf("EMG start failed\r\n");
      osMutexRelease(uart2MutexHandle);
    }
  }

  /* Infinite loop */
  for(;;)
  {
    // 闈為樆濉炶幏鍙栧崐缂撳啿淇″彿閲忥紝鏈変俊鍙峰氨澶勭悊
    if (osSemaphoreAcquire(adcHalfCpltSemHandle, 0U) == osOK)
    {
      EmgSensor_ProcessHalfBuffer();
    }

    // 闈為樆濉炶幏鍙栧叏缂撳啿淇″彿閲?
    if (osSemaphoreAcquire(adcCpltSemHandle, 0U) == osOK)
    {
      EmgSensor_ProcessFullBuffer();
    }
    // 200ms 鍛ㄦ湡鐘舵€佹墦鍗帮紙璋冭瘯鐢級
//    if ((osKernelGetTickCount() - last_print_tick) >= 200U)
//    {
//      last_print_tick = osKernelGetTickCount();
//      EmgSensor_GetFeatures(&emg_features);
//      model_input.raw = emg_features.raw;
//      model_input.baseline_x10 = emg_features.baseline_x10;
//      model_input.drop_x10 = emg_features.drop_x10;
//      model_input.rectified_x10 = emg_features.rectified_x10;
//      model_input.envelope_x10 = emg_features.envelope_x10;
//      model_input.rms_x10 = emg_features.rms_x10;

//      osMutexAcquire(uart2MutexHandle, osWaitForever);
//      if (EMG_UART4_RECORD_MODE != 0U)
//      {
//        if (emg_features.calibrated == 0U)
//        {
//          printf("EMGPRED cal=0 raw=%u base_x10=%ld\r\n",
//                 (unsigned int)emg_features.raw,
//                 (long)emg_features.baseline_x10);
//        }
//        else
//        {
//          emg_action = EmgRfModel_Predict(&model_input, &confidence_x1000);
//          force_percent = Emg_EstimateForcePercent(&emg_features);
//          flex_level = 0U;
//          if (emg_action == EMG_ACTION_ARM_FLEX_LIGHT)
//          {
//            flex_level = 1U;
//          }
//          else if (emg_action == EMG_ACTION_ARM_FLEX_STRONG)
//          {
//            flex_level = 2U;
//          }

//          printf("EMGPRED action=%s force=%u raw=%u drop_x10=%ld env_x10=%ld rms_x10=%ld\r\n",
//                 EmgRfModel_LabelName(emg_action),
//                 (unsigned int)force_percent,
//                 (unsigned int)emg_features.raw,
//                 (long)emg_features.drop_x10,
//                 (long)emg_features.envelope_x10,
//                 (long)emg_features.rms_x10);
//        }
//      }
//      else
//      {
//        emg_action = EmgRfModel_Predict(&model_input, &confidence_x1000);
//        force_percent = Emg_EstimateForcePercent(&emg_features);
//        printf("EMGDBGv4 raw=%u base_x10=%ld drop_x10=%ld rect_x10=%ld env_x10=%ld rms_x10=%ld strength=%u force=%u cal=%u action=%s conf=%u feature_size=%u\r\n",
//               (unsigned int)emg_features.raw,
//               (long)emg_features.baseline_x10,
//               (long)emg_features.drop_x10,
//               (long)emg_features.rectified_x10,
//               (long)emg_features.envelope_x10,
//               (long)emg_features.rms_x10,
//               (unsigned int)emg_features.strength,
//               (unsigned int)force_percent,
//               (unsigned int)emg_features.calibrated,
//               EmgRfModel_LabelName(emg_action),
//               (unsigned int)confidence_x1000,
//               (unsigned int)sizeof(EmgSensor_Features_t));
//      }
//      osMutexRelease(uart2MutexHandle);
//    }

    osDelay(1);
  }
  /* USER CODE END EmgTask */
}

/* USER CODE BEGIN Header_StartWirelessTask */
/**
* @brief Function implementing the WirelessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWirelessTask */
//  杞鎺ユ敹鏃犵嚎鎵嬭厱绔笅鍙戠殑涓插彛甯?
void StartWirelessTask(void *argument)
{
  /* USER CODE BEGIN WirelessTask */
  uint8_t rx_byte;
  WirelessWristFrame_t frame;

  (void)argument;
  WirelessLink_Init();
  printf("JDY USART1 task start, baud=115200\r\n");

  /* Infinite loop */
  for(;;)
  {
    /* 本任务专职接收腕端帧：逐字节阻塞读 USART1 喂给解析器，
       解析器内部在校验通过时更新 latest_frame，供其他任务用
       WirelessLink_GetLatest 取最新姿态。 */
    if (HAL_UART_Receive(&huart1, &rx_byte, 1U, 100U) == HAL_OK)
    {
      (void)WirelessLink_PushByte(rx_byte, &frame);
    }
  }
  /* USER CODE END WirelessTask */
}

/* USER CODE BEGIN Header_StartDataStreamTask */
/**
* @brief Function implementing the DataStreamTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataStreamTask */
void StartDataStreamTask(void *argument)
{
  /* USER CODE BEGIN DataStreamTask */
  EmgSensor_Features_t emg_features;
  char line[128];
  int len = 0;

  (void)argument;

  if (EMG_UART4_RECORD_MODE == 0U)
  {
    osMutexAcquire(uart2MutexHandle, osWaitForever);
//    printf("Data stream USART2 PA2(TX) PA3(RX) start\r\n");
    osMutexRelease(uart2MutexHandle);
  }
  /* Infinite loop */
  for(;;)
  {
    EmgSensor_GetFeatures(&emg_features);

//    len = snprintf(line,
//                   sizeof(line),
//                   "%u %ld %ld %ld %ld %ld\r\n",
//                   (unsigned int)emg_features.raw,
//                   (long)emg_features.baseline_x10,
//                   (long)emg_features.drop_x10,
//                   (long)emg_features.rectified_x10,
//                   (long)emg_features.envelope_x10,
//                   (long)emg_features.rms_x10);

    if ((EMG_UART4_RECORD_MODE == 0U) && (len > 0))
    {
      (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100U);
    }

    osDelay(DATA_STREAM_PERIOD_MS);
  }
  /* USER CODE END DataStreamTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
* @brief Function implementing the UiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUiTask */
void StartUiTask(void *argument)
{
  /* USER CODE BEGIN UiTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UiTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN DisplayTask */
  // LCD_Touch_TestTask();//娴嬭瘯

  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END DisplayTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN DebugTask */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END DebugTask */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

