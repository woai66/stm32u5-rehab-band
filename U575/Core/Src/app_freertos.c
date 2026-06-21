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
#include "lcd_st7789.h"
#include "lsm6dsr.h"
#include "imu_processing.h"
#include "i2c.h"
#include "spi.h"
#include "touch_cst816t.h"
#include "usart.h"
#include "wireless_link.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_ATTITUDE_KP    (1.0f)
#define IMU_ATTITUDE_KI    (0.05f)
#define IMU_ANGLE_DEADBAND_X10  (5)
#define IMU_GYRO_DEADBAND_RAW  (2)
#define WRIST_WIRELESS_PERIOD_MS  (50U)
/* 置 1 时 DebugTask 经 UART4 打印实时姿态角（带符号，单位 0.1°），仅调试用 */
#define DEBUG_ANGLE_PRINT  (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Wrist node local IMU cache. This node sends these values to the upper-arm node. */
static LSM6DSR_Data_t wrist_imu_raw;
static IMUProc_Euler_t wrist_imu_euler;
static uint8_t wrist_imu_valid;

/* USER CODE END Variables */

/* USER CODE BEGIN 0 */
static int16_t AngleToX100(float angle_deg)
{
  int32_t scaled;

  scaled = (int32_t)((angle_deg * 100.0f) + ((angle_deg >= 0.0f) ? 0.5f : -0.5f));
  if (scaled > INT16_MAX)
  {
    scaled = INT16_MAX;
  }
  else if (scaled < INT16_MIN)
  {
    scaled = INT16_MIN;
  }

  return (int16_t)scaled;
}

static int16_t FloatToI16Scaled(float value, float scale)
{
  int32_t scaled;

  scaled = (int32_t)((value * scale) + ((value >= 0.0f) ? 0.5f : -0.5f));
  if (scaled > INT16_MAX)
  {
    scaled = INT16_MAX;
  }
  else if (scaled < INT16_MIN)
  {
    scaled = INT16_MIN;
  }

  return (int16_t)scaled;
}

static void FillWristWirelessFrame(WirelessWristFrame_t *frame,
                                   uint16_t seq,
                                   uint32_t tick,
                                   const LSM6DSR_Data_t *imu_raw,
                                   const IMUProc_Euler_t *euler,
                                   uint8_t imu_valid)
{
  frame->seq = seq;
  frame->tick = tick;
  frame->acc_mg[0] = FloatToI16Scaled(imu_raw->acc_g_x, 1000.0f);
  frame->acc_mg[1] = FloatToI16Scaled(imu_raw->acc_g_y, 1000.0f);
  frame->acc_mg[2] = FloatToI16Scaled(imu_raw->acc_g_z, 1000.0f);
  frame->gyro_dps_x10[0] = FloatToI16Scaled(imu_raw->gyro_dps_x, 10.0f);
  frame->gyro_dps_x10[1] = FloatToI16Scaled(imu_raw->gyro_dps_y, 10.0f);
  frame->gyro_dps_x10[2] = FloatToI16Scaled(imu_raw->gyro_dps_z, 10.0f);
  frame->angle_x100[0] = AngleToX100(euler->roll_deg);
  frame->angle_x100[1] = AngleToX100(euler->pitch_deg);
  frame->angle_x100[2] = AngleToX100(euler->yaw_deg);
  frame->status = (imu_valid != 0U) ? 0x01U : 0x00U;
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
  .stack_size = 512 * 4
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
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024 * 4
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

  /* Wrist node has no EMG sensor. Keep the task entry generated, but do not start it. */
  EmgTaskHandle = NULL;

  /* HC-05 transparent UART link to the upper-arm node. */
  WirelessTaskHandle = osThreadNew(StartWirelessTask, NULL, &WirelessTask_attributes);

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
  /* This task samples and solves the wrist IMU. */
  LSM6DSR_Data_t imu_data;
  IMUProc_State_t imu_state;
  int32_t gyro_bias_raw[3] = {0};
  int32_t gyro_bias_sum[3] = {0};
  uint32_t last_error_tick = 0U;
  uint16_t calib_count = 0U;

  IMUProc_StateInit(&imu_state, 0.2f);
  IMUProc_AttitudeSetGains(&imu_state.attitude, IMU_ATTITUDE_KP, IMU_ATTITUDE_KI);

  while (LSM6DSR_Init() != HAL_OK)
  {
    if ((osKernelGetTickCount() - last_error_tick) >= 500U)
    {
      last_error_tick = osKernelGetTickCount();
      osMutexAcquire(uart2MutexHandle, osWaitForever);
      printf("LSM start failed, WHO_AM_I=0x%02X\r\n", LSM6DSR_ReadWhoAmI());
      osMutexRelease(uart2MutexHandle);
    }
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    osDelay(100);
  }

  osMutexAcquire(uart2MutexHandle, osWaitForever);
  printf("Wrist IMU start OK\r\n");
  printf("Keep wrist IMU still for gyro calibration\r\n");
  osMutexRelease(uart2MutexHandle);

  for (uint16_t i = 0U; i < 100U; i++)
  {
    (void)LSM6DSR_ReadRaw(&imu_data);
    osDelay(5);
  }

  IMUProc_CalibrationReset(&imu_state.calib);
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

  osMutexAcquire(uart2MutexHandle, osWaitForever);
//  printf("Wrist IMU calib OK, gyro_bias_raw=%d,%d,%d sample_raw=%d,%d,%d\r\n",
//         (int)gyro_bias_raw[0],
//         (int)gyro_bias_raw[1],
//         (int)gyro_bias_raw[2],
//         (int)imu_data.gyro_x,
//         (int)imu_data.gyro_y,
//         (int)imu_data.gyro_z);
  osMutexRelease(uart2MutexHandle);

  /* Infinite loop */
  for(;;)
  {
    if (LSM6DSR_ReadRaw(&imu_data) == HAL_OK)
    {
      IMUProc_PrepareRawForUpdate(&imu_data, gyro_bias_raw, IMU_GYRO_DEADBAND_RAW);
      IMUProc_StateUpdate(&imu_state, &imu_data, 0.005f);

      taskENTER_CRITICAL();
      /* Publish wrist IMU data for debug, display, and wireless upload to the upper-arm node. */
      wrist_imu_raw = imu_data;
      wrist_imu_euler = imu_state.attitude.euler;
      wrist_imu_valid = 1U;
      taskEXIT_CRITICAL();
    }
    else
    {
      if ((osKernelGetTickCount() - last_error_tick) >= 500U)
      {
        last_error_tick = osKernelGetTickCount();
        osMutexAcquire(uart2MutexHandle, osWaitForever);
        printf("LSM read failed\r\n");
        osMutexRelease(uart2MutexHandle);
      }
      HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
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
void StartEmgTask(void *argument)
{
  /* USER CODE BEGIN EmgTask */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
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
void StartWirelessTask(void *argument)
{
  /* USER CODE BEGIN WirelessTask */
  uint8_t tx_buf[WIRELESS_WRIST_FRAME_SIZE];
  WirelessWristFrame_t wrist_frame;
  LSM6DSR_Data_t imu_raw;
  IMUProc_Euler_t imu_euler;
  uint16_t seq = 0U;
  uint32_t last_debug_tick = 0U;
  uint32_t tx_count = 0U;
  uint8_t imu_valid;

  (void)argument;
  WirelessLink_Init();

  printf("HC05 USART1 TX task start, baud=9600\r\n");

  /* Infinite loop */
  for(;;)
  {
    taskENTER_CRITICAL();
    imu_raw = wrist_imu_raw;
    imu_euler = wrist_imu_euler;
    imu_valid = wrist_imu_valid;
    taskEXIT_CRITICAL();

    /* Keep sending heartbeat frames even before the IMU becomes valid.
       This lets the upper-arm node verify the HC-05 UART link independently. */
    FillWristWirelessFrame(&wrist_frame,
                           seq++,
                           osKernelGetTickCount(),
                           &imu_raw,
                           &imu_euler,
                           imu_valid);

    if (WirelessLink_BuildWristFrame(tx_buf, &wrist_frame) != 0U)
    {
      osMutexAcquire(uart1MutexHandle, osWaitForever);
      if (HAL_UART_Transmit(&huart1, tx_buf, WIRELESS_WRIST_FRAME_SIZE, 50U) == HAL_OK)
      {
        tx_count++;
      }
      osMutexRelease(uart1MutexHandle);
    }

    if ((osKernelGetTickCount() - last_debug_tick) >= 1000U)
    {
      last_debug_tick = osKernelGetTickCount();
      printf("HC05_TX count=%lu seq=%u valid=%u acc_mg=%d,%d,%d gyro_x10=%d,%d,%d angle_x100=%d,%d,%d\r\n",
             (unsigned long)tx_count,
             (unsigned int)wrist_frame.seq,
             (unsigned int)imu_valid,
             (int)wrist_frame.acc_mg[0],
             (int)wrist_frame.acc_mg[1],
             (int)wrist_frame.acc_mg[2],
             (int)wrist_frame.gyro_dps_x10[0],
             (int)wrist_frame.gyro_dps_x10[1],
             (int)wrist_frame.gyro_dps_x10[2],
             (int)wrist_frame.angle_x100[0],
             (int)wrist_frame.angle_x100[1],
             (int)wrist_frame.angle_x100[2]);
    }

    osDelay(WRIST_WIRELESS_PERIOD_MS);
  }
  /* USER CODE END WirelessTask */
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
  (void)argument;

//  osMutexAcquire(uart2MutexHandle, osWaitForever);
//  printf("LCD backlight blink test start\r\n");
//  osMutexRelease(uart2MutexHandle);

  for (uint8_t i = 0U; i < 3U; i++)
  {
    LCD_SetBacklight(1U);
    osDelay(250);
    LCD_SetBacklight(0U);
    osDelay(250);
  }

  LCD_SetBacklight(1U);
  osDelay(300);

//  osMutexAcquire(uart2MutexHandle, osWaitForever);
//  printf("LCD touch test start\r\n");
//  osMutexRelease(uart2MutexHandle);

  {
    HAL_StatusTypeDef lcd_status;
    HAL_StatusTypeDef touch_status = HAL_OK;

    lcd_status = LCD_Init();
    touch_status = CST816T_Init();

//    osMutexAcquire(uart2MutexHandle, osWaitForever);
//    printf("LCD init status=%d spi_status=%d spi_err=0x%08lX spi_state=%u touch_status=%d i2c_err=0x%08lX\r\n",
//           (int)lcd_status,
//           (int)LCD_GetLastStatus(),
//           (unsigned long)hspi3.ErrorCode,
//           (unsigned int)hspi3.State,
//           (int)touch_status,
//           (unsigned long)hi2c1.ErrorCode);
//    osMutexRelease(uart2MutexHandle);
  }

  LCD_FillScreen(LCD_COLOR_BLACK);
  /* 行首色块标识：红=Roll 绿=Pitch 蓝=Yaw */
  LCD_Fill(6U, 20U, 24U, 66U, LCD_COLOR_RED);
  LCD_Fill(6U, 110U, 24U, 156U, LCD_COLOR_GREEN);
  LCD_Fill(6U, 200U, 24U, 246U, LCD_COLOR_BLUE);

  /* 上一帧值，仅在变化时重绘对应行，避免静止时无谓刷新 */
  int32_t last_roll = INT32_MIN;
  int32_t last_pitch = INT32_MIN;
  int32_t last_yaw = INT32_MIN;

  for(;;)
  {
    IMUProc_Euler_t euler;
    IMUProc_EulerX10_t euler_x10;
    uint8_t valid;

    taskENTER_CRITICAL();
    euler = wrist_imu_euler;
    valid = wrist_imu_valid;
    taskEXIT_CRITICAL();

    if (valid != 0U)
    {
      IMUProc_EulerToSignedX10(&euler, IMU_ANGLE_DEADBAND_X10, &euler_x10);
      if (euler_x10.roll_x10 != last_roll)
      {
        (void)LCD_DrawNumberX10(34U, 20U, 30U, 46U, 6U, euler_x10.roll_x10, LCD_COLOR_RED, LCD_COLOR_BLACK);
        last_roll = euler_x10.roll_x10;
      }
      if (euler_x10.pitch_x10 != last_pitch)
      {
        (void)LCD_DrawNumberX10(34U, 110U, 30U, 46U, 6U, euler_x10.pitch_x10, LCD_COLOR_GREEN, LCD_COLOR_BLACK);
        last_pitch = euler_x10.pitch_x10;
      }
      if (euler_x10.yaw_x10 != last_yaw)
      {
        (void)LCD_DrawNumberX10(34U, 200U, 30U, 46U, 6U, euler_x10.yaw_x10, LCD_COLOR_BLUE, LCD_COLOR_BLACK);
        last_yaw = euler_x10.yaw_x10;
      }
    }
    osDelay(50);
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
#if DEBUG_ANGLE_PRINT
    LSM6DSR_Data_t imu_data;
    IMUProc_Euler_t imu_euler;
    IMUProc_EulerX10_t imu_euler_x10;
    uint8_t imu_valid;

    taskENTER_CRITICAL();
    imu_data = wrist_imu_raw;
    imu_euler = wrist_imu_euler;
    imu_valid = wrist_imu_valid;
    taskEXIT_CRITICAL();

    if (imu_valid != 0U)
    {
      int32_t ax[3];
      const char *sgn[3];
      uint32_t mag[3];
      uint8_t i;

      IMUProc_EulerToSignedX10(&imu_euler, IMU_ANGLE_DEADBAND_X10, &imu_euler_x10);
      ax[0] = imu_euler_x10.roll_x10;
      ax[1] = imu_euler_x10.pitch_x10;
      ax[2] = imu_euler_x10.yaw_x10;
      for (i = 0U; i < 3U; i++)
      {
          sgn[i] = (ax[i] < 0) ? "-" : "";
          mag[i] = (ax[i] < 0) ? (uint32_t)(-ax[i]) : (uint32_t)ax[i];
      }
      (void)imu_data;

      osMutexAcquire(uart2MutexHandle, osWaitForever);
      printf("ANGLE roll=%s%lu.%lu pitch=%s%lu.%lu yaw=%s%lu.%lu\r\n",
             sgn[0], (unsigned long)(mag[0] / 10U), (unsigned long)(mag[0] % 10U),
             sgn[1], (unsigned long)(mag[1] / 10U), (unsigned long)(mag[1] % 10U),
             sgn[2], (unsigned long)(mag[2] / 10U), (unsigned long)(mag[2] % 10U));
      osMutexRelease(uart2MutexHandle);
    }
#endif
    osDelay(100);
  }
  /* USER CODE END DebugTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

