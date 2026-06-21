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
#include "imu_processing.h"
#include "rehab_eval.h"
#include "usart.h"
#include "wireless_link.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  REHAB_ACTION_IDLE = 0,
  REHAB_ACTION_ELBOW_FLEX,
  REHAB_ACTION_ARM_ABDUCTION,
  REHAB_ACTION_FOREARM_ROTATION,
  REHAB_ACTION_SHOULDER_LIFT
} RehabAction_t;

typedef struct
{
  int16_t upper_acc_mg[3];
  int16_t upper_gyro_x10[3];
  int16_t upper_angle_x100[3];
  int16_t wrist_acc_mg[3];
  int16_t wrist_gyro_x10[3];
  int16_t wrist_angle_x100[3];
  int32_t elbow_relative_pitch_x100;
  int32_t elbow_angle_x100;
  int32_t forearm_rotation_angle_x100;
  int32_t arm_abduction_angle_x100;
  int32_t shoulder_lift_angle_x100;
  int32_t elbow_velocity_x100_s;
  int32_t elbow_range_x100;
  int32_t emg_rms_x10;
  uint8_t level;
  uint8_t emg_active;
  uint8_t valid;
  RehabAction_t action;
} RehabFrame_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_ATTITUDE_KP    (1.0f)
#define IMU_ATTITUDE_KI    (0.05f)
#define IMU_ANGLE_DEADBAND_X10  (5)
#define IMU_GYRO_DEADBAND_RAW  (2)
#define DATA_STREAM_PERIOD_MS  (20U)
#define EMG_UART4_RECORD_MODE  (1U)
#define UPPER_GYRO_DEBUG_PRINT  (0U)
#define REHAB_ALGO_PERIOD_MS  (100U)
#define REHAB_WINDOW_MS       (1000U)
#define REHAB_ELBOW_RANGE_FLEX_X100       (2500)
#define REHAB_ELBOW_VEL_FLEX_X100_S       (2500)
#define REHAB_ELBOW_HOLD_ENTER_X100       (3000)
#define REHAB_ELBOW_HOLD_EXIT_X100        (3000)
#define REHAB_FOREARM_ROT_HOLD_ENTER_X100 (1500)
#define REHAB_FOREARM_ROT_HOLD_EXIT_X100  (800)
#define REHAB_FOREARM_ROT_STRONG_X100     (3000)
#define REHAB_ARM_ABD_HOLD_ENTER_X100     (2000)
#define REHAB_ARM_ABD_HOLD_EXIT_X100      (1800)
#define REHAB_ARM_ABD_MIN_SHOULDER_X100   (2000)
#define REHAB_ARM_ABD_MIN_SHOULDER_EXIT_X100 (1800)
#define REHAB_SHOULDER_HOLD_ENTER_X100    (1800)
#define REHAB_SHOULDER_HOLD_EXIT_X100     (1000)
#define REHAB_ACTION_LATCH_MS             (1500U)
#define REHAB_WRIST_ROT_GYRO_X10          (200)
#define REHAB_FOREARM_UPPER_STABLE_GYRO_X10 (300)
#define REHAB_FOREARM_UPPER_STABLE_ANGLE_X100 (2000)
#define REHAB_ARM_ABDUCTION_GYRO_X10      (250)
#define REHAB_SHOULDER_LIFT_GYRO_X10      (80)
#define REHAB_ACTIVE_EVAL_ELBOW_FLEX      (1U)
#define REHAB_ACTIVE_EVAL_FOREARM_ROT     (2U)
#define REHAB_ACTIVE_EVAL_ARM_ABD         (3U)
#define REHAB_ACTIVE_EVAL_SHOULDER_LIFT   (4U)
#define REHAB_ACTIVE_EVAL                 REHAB_ACTIVE_EVAL_SHOULDER_LIFT
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
static WirelessWristFrame_t latest_wrist_frame;
static uint8_t latest_wrist_valid;
static RehabFrame_t latest_rehab_frame;
static uint8_t latest_rehab_valid;

/* USART1 单字节中断接收缓冲，喂给腕端无线帧解析器 */
static uint8_t wireless_rx_byte;

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

static int32_t Rehab_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int16_t Rehab_FloatToI16Scaled(float value, float scale)
{
  int32_t scaled = (int32_t)((value * scale) + ((value >= 0.0f) ? 0.5f : -0.5f));

  if (scaled > INT16_MAX)
  {
    scaled = INT16_MAX;
  }
  if (scaled < INT16_MIN)
  {
    scaled = INT16_MIN;
  }
  return (int16_t)scaled;
}

static int16_t Rehab_AngleToSignedX100(float angle_deg)
{
  int32_t scaled;

  while (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }
  while (angle_deg >= 180.0f)
  {
    angle_deg -= 360.0f;
  }

  scaled = (int32_t)((angle_deg * 100.0f) + ((angle_deg >= 0.0f) ? 0.5f : -0.5f));
  if (scaled > INT16_MAX)
  {
    scaled = INT16_MAX;
  }
  if (scaled < INT16_MIN)
  {
    scaled = INT16_MIN;
  }
  return (int16_t)scaled;
}

static int32_t Rehab_AngleDiffAbsX100(int32_t a_x100, int32_t b_x100)
{
  int32_t diff = a_x100 - b_x100;

  while (diff > 18000)
  {
    diff -= 36000;
  }
  while (diff < -18000)
  {
    diff += 36000;
  }

  return Rehab_AbsI32(diff);
}

static int32_t Rehab_AngleDiffSignedX100(int32_t a_x100, int32_t b_x100)
{
  int32_t diff = a_x100 - b_x100;

  while (diff > 18000)
  {
    diff -= 36000;
  }
  while (diff < -18000)
  {
    diff += 36000;
  }

  return diff;
}

static char Rehab_SignCharI32(int32_t value)
{
  return (value < 0) ? '-' : '+';
}

static int32_t Rehab_ComputeElbowFlexionX100(int32_t upper_pitch_x100,
                                             int32_t wrist_pitch_x100,
                                             int32_t zero_relative_pitch_x100,
                                             int32_t *relative_pitch_x100)
{
  int32_t relative;
  int32_t flexion;

  relative = Rehab_AngleDiffSignedX100(wrist_pitch_x100, upper_pitch_x100);
  flexion = Rehab_AbsI32(Rehab_AngleDiffSignedX100(relative, zero_relative_pitch_x100));
  if (flexion > 18000)
  {
    flexion = 18000;
  }

  if (relative_pitch_x100 != NULL)
  {
    *relative_pitch_x100 = relative;
  }

  return flexion;
}

static const char *Rehab_ActionName(RehabAction_t action)
{
  switch (action)
  {
    case REHAB_ACTION_ELBOW_FLEX:
      return "ELBOW_FLEX";
    case REHAB_ACTION_ARM_ABDUCTION:
      return "ARM_ABD";
    case REHAB_ACTION_FOREARM_ROTATION:
      return "FOREARM_ROT";
    case REHAB_ACTION_SHOULDER_LIFT:
      return "SHOULDER_LIFT";
    case REHAB_ACTION_IDLE:
    default:
      return "IDLE";
  }
}

static RehabAction_t Rehab_ClassifyRule(const RehabFrame_t *frame, RehabAction_t previous_action)
{
  int32_t wrist_gx_abs;
  int32_t upper_gy_abs;
  int32_t upper_gz_abs;
  uint8_t upper_stable_for_forearm;
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  uint8_t forearm_motion_hint;
#endif

  if ((frame == NULL) || (frame->valid == 0U))
  {
    return REHAB_ACTION_IDLE;
  }

  wrist_gx_abs = Rehab_AbsI32(frame->wrist_gyro_x10[0]);
  upper_gy_abs = Rehab_AbsI32(frame->upper_gyro_x10[1]);
  upper_gz_abs = Rehab_AbsI32(frame->upper_gyro_x10[2]);
  upper_stable_for_forearm =
      ((upper_gy_abs < REHAB_FOREARM_UPPER_STABLE_GYRO_X10) &&
       (upper_gz_abs < REHAB_FOREARM_UPPER_STABLE_GYRO_X10) &&
       (frame->arm_abduction_angle_x100 < REHAB_FOREARM_UPPER_STABLE_ANGLE_X100) &&
       (frame->shoulder_lift_angle_x100 < REHAB_FOREARM_UPPER_STABLE_ANGLE_X100)) ? 1U : 0U;

#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  forearm_motion_hint =
      ((wrist_gx_abs >= REHAB_WRIST_ROT_GYRO_X10) ||
       (frame->forearm_rotation_angle_x100 >= REHAB_FOREARM_ROT_HOLD_ENTER_X100) ||
       ((previous_action == REHAB_ACTION_FOREARM_ROTATION) &&
        (frame->forearm_rotation_angle_x100 >= REHAB_FOREARM_ROT_HOLD_EXIT_X100))) ? 1U : 0U;

  if (frame->forearm_rotation_angle_x100 >= REHAB_FOREARM_ROT_STRONG_X100)
  {
    return REHAB_ACTION_FOREARM_ROTATION;
  }

  if ((forearm_motion_hint != 0U) && (upper_stable_for_forearm != 0U))
  {
    return REHAB_ACTION_FOREARM_ROTATION;
  }

  if (forearm_motion_hint == 0U)
  {
#endif
  if ((frame->elbow_angle_x100 >= REHAB_ELBOW_HOLD_ENTER_X100) ||
      ((previous_action == REHAB_ACTION_ELBOW_FLEX) &&
       (frame->elbow_angle_x100 >= REHAB_ELBOW_HOLD_EXIT_X100)))
  {
    return REHAB_ACTION_ELBOW_FLEX;
  }

  if ((frame->elbow_range_x100 >= REHAB_ELBOW_RANGE_FLEX_X100) ||
      (Rehab_AbsI32(frame->elbow_velocity_x100_s) >= REHAB_ELBOW_VEL_FLEX_X100_S))
  {
    return REHAB_ACTION_ELBOW_FLEX;
  }
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  }
#endif
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_SHOULDER_LIFT)
  if (upper_gy_abs >= REHAB_SHOULDER_LIFT_GYRO_X10)
  {
    return REHAB_ACTION_SHOULDER_LIFT;
  }
  if ((frame->shoulder_lift_angle_x100 >= REHAB_SHOULDER_HOLD_ENTER_X100) ||
      ((previous_action == REHAB_ACTION_SHOULDER_LIFT) &&
       (frame->shoulder_lift_angle_x100 >= REHAB_SHOULDER_HOLD_EXIT_X100)))
  {
    return REHAB_ACTION_SHOULDER_LIFT;
  }
#endif
  if (upper_gz_abs >= REHAB_ARM_ABDUCTION_GYRO_X10)
  {
    return REHAB_ACTION_ARM_ABDUCTION;
  }
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_ARM_ABD)
  if ((frame->arm_abduction_angle_x100 >= REHAB_ARM_ABD_HOLD_ENTER_X100) ||
      ((previous_action == REHAB_ACTION_ARM_ABDUCTION) &&
       (frame->arm_abduction_angle_x100 >= REHAB_ARM_ABD_HOLD_EXIT_X100)))
  {
    return REHAB_ACTION_ARM_ABDUCTION;
  }
#else
  if (((frame->arm_abduction_angle_x100 >= REHAB_ARM_ABD_HOLD_ENTER_X100) &&
       (frame->shoulder_lift_angle_x100 >= REHAB_ARM_ABD_MIN_SHOULDER_X100)) ||
      ((previous_action == REHAB_ACTION_ARM_ABDUCTION) &&
       (frame->arm_abduction_angle_x100 >= REHAB_ARM_ABD_HOLD_EXIT_X100) &&
       (frame->shoulder_lift_angle_x100 >= REHAB_ARM_ABD_MIN_SHOULDER_EXIT_X100)))
  {
    return REHAB_ACTION_ARM_ABDUCTION;
  }
#endif

#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  if (forearm_motion_hint == 0U)
  {
#endif
    if (upper_gy_abs >= REHAB_SHOULDER_LIFT_GYRO_X10)
    {
      return REHAB_ACTION_SHOULDER_LIFT;
    }
    if ((frame->shoulder_lift_angle_x100 >= REHAB_SHOULDER_HOLD_ENTER_X100) ||
        ((previous_action == REHAB_ACTION_SHOULDER_LIFT) &&
         (frame->shoulder_lift_angle_x100 >= REHAB_SHOULDER_HOLD_EXIT_X100)))
    {
      return REHAB_ACTION_SHOULDER_LIFT;
    }
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  }
#endif

  if ((wrist_gx_abs >= REHAB_WRIST_ROT_GYRO_X10) &&
      (upper_stable_for_forearm != 0U))
  {
    return REHAB_ACTION_FOREARM_ROTATION;
  }
  if ((upper_stable_for_forearm != 0U) &&
      ((frame->forearm_rotation_angle_x100 >= REHAB_FOREARM_ROT_HOLD_ENTER_X100) ||
       ((previous_action == REHAB_ACTION_FOREARM_ROTATION) &&
        (frame->forearm_rotation_angle_x100 >= REHAB_FOREARM_ROT_HOLD_EXIT_X100))))
  {
    return REHAB_ACTION_FOREARM_ROTATION;
  }

  return REHAB_ACTION_IDLE;
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
#if SENSOR_GYRO_DEBUG_PRINT
  uint32_t last_gyro_print_tick = 0U;
#endif
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

      if ((UPPER_GYRO_DEBUG_PRINT != 0U) && ((osKernelGetTickCount() - last_gyro_print_tick) >= 200U))
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
  LSM6DSR_Data_t upper_raw;
  IMUProc_Euler_t upper_euler;
  WirelessWristFrame_t wrist_frame;
  EmgSensor_Features_t emg_features;
  RehabFrame_t rehab_frame;
  uint8_t upper_valid;
  uint8_t wrist_valid;
  uint32_t now_tick;
  uint32_t last_tick = 0U;
  uint32_t window_start_tick = 0U;
  uint32_t last_print_tick = 0U;
  uint32_t last_train_print_tick = 0U;
  int32_t last_elbow_angle_x100 = 0;
  int32_t window_min_elbow_x100 = 0;
  int32_t window_max_elbow_x100 = 0;
  int32_t elbow_zero_relative_pitch_x100 = 0;
  int32_t forearm_zero_roll_x100 = 0;
  int32_t arm_abd_zero_yaw_x100 = 0;
  int32_t shoulder_zero_pitch_x100 = 0;
  uint8_t have_last_angle = 0U;
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  uint8_t forearm_upper_stable = 0U;
#endif
  uint8_t window_initialized = 0U;
  uint8_t elbow_zero_valid = 0U;
  RehabAction_t previous_rule_action = REHAB_ACTION_IDLE;
  RehabAction_t latched_action = REHAB_ACTION_IDLE;
  RehabAction_t rule_action = REHAB_ACTION_IDLE;
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  ForearmRotEvalConfig_t forearm_eval_config;
  ForearmRotEvalSample_t forearm_eval_sample;
  ForearmRotEvalState_t forearm_eval_state;
#elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_ARM_ABD)
  ArmAbdEvalConfig_t arm_abd_eval_config;
  ArmAbdEvalSample_t arm_abd_eval_sample;
  ArmAbdEvalState_t arm_abd_eval_state;
#elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_SHOULDER_LIFT)
  ShoulderLiftEvalConfig_t shoulder_lift_eval_config;
  ShoulderLiftEvalSample_t shoulder_lift_eval_sample;
  ShoulderLiftEvalState_t shoulder_lift_eval_state;
#else
  ElbowFlexEvalConfig_t elbow_eval_config;
  ElbowFlexEvalSample_t elbow_eval_sample;
  ElbowFlexEvalState_t elbow_eval_state;
#endif
  uint32_t latched_action_tick = 0U;

  (void)argument;
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
  ForearmRotEval_Init();
  ForearmRotEval_DefaultConfig(&forearm_eval_config);
  ForearmRotEval_Start(&forearm_eval_config);
  printf("TRAIN_START FOREARM_ROT target=%u\r\n",
         (unsigned int)forearm_eval_config.target_reps);
#elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_ARM_ABD)
  ArmAbdEval_Init();
  ArmAbdEval_DefaultConfig(&arm_abd_eval_config);
  ArmAbdEval_Start(&arm_abd_eval_config);
  printf("TRAIN_START ARM_ABD target=%u\r\n",
         (unsigned int)arm_abd_eval_config.target_reps);
#elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_SHOULDER_LIFT)
  ShoulderLiftEval_Init();
  ShoulderLiftEval_DefaultConfig(&shoulder_lift_eval_config);
  ShoulderLiftEval_Start(&shoulder_lift_eval_config);
  printf("TRAIN_START SHOULDER_LIFT target=%u\r\n",
         (unsigned int)shoulder_lift_eval_config.target_reps);
#else
  ElbowFlexEval_Init();
  ElbowFlexEval_DefaultConfig(&elbow_eval_config);
  ElbowFlexEval_Start(&elbow_eval_config);
  printf("TRAIN_START ELBOW_FLEX target=%u\r\n",
         (unsigned int)elbow_eval_config.target_reps);
#endif

  /* Infinite loop */
  for(;;)
  {
    taskENTER_CRITICAL();
    upper_raw = debug_imu_raw;
    upper_euler = debug_imu_euler;
    upper_valid = debug_imu_valid;
    wrist_frame = latest_wrist_frame;
    wrist_valid = latest_wrist_valid;
    taskEXIT_CRITICAL();

    EmgSensor_GetFeatures(&emg_features);
    now_tick = osKernelGetTickCount();

    memset(&rehab_frame, 0, sizeof(rehab_frame));
    rehab_frame.valid = ((upper_valid != 0U) &&
                         (wrist_valid != 0U) &&
                         ((wrist_frame.status & 0x01U) != 0U)) ? 1U : 0U;

    if (rehab_frame.valid != 0U)
    {
      rehab_frame.upper_acc_mg[0] = Rehab_FloatToI16Scaled(upper_raw.acc_g_x, 1000.0f);
      rehab_frame.upper_acc_mg[1] = Rehab_FloatToI16Scaled(upper_raw.acc_g_y, 1000.0f);
      rehab_frame.upper_acc_mg[2] = Rehab_FloatToI16Scaled(upper_raw.acc_g_z, 1000.0f);
      rehab_frame.upper_gyro_x10[0] = Rehab_FloatToI16Scaled(upper_raw.gyro_dps_x, 10.0f);
      rehab_frame.upper_gyro_x10[1] = Rehab_FloatToI16Scaled(upper_raw.gyro_dps_y, 10.0f);
      rehab_frame.upper_gyro_x10[2] = Rehab_FloatToI16Scaled(upper_raw.gyro_dps_z, 10.0f);
      rehab_frame.upper_angle_x100[0] = Rehab_AngleToSignedX100(upper_euler.roll_deg);
      rehab_frame.upper_angle_x100[1] = Rehab_AngleToSignedX100(upper_euler.pitch_deg);
      rehab_frame.upper_angle_x100[2] = Rehab_AngleToSignedX100(upper_euler.yaw_deg);

      for (uint8_t i = 0U; i < 3U; i++)
      {
        rehab_frame.wrist_acc_mg[i] = wrist_frame.acc_mg[i];
        rehab_frame.wrist_gyro_x10[i] = wrist_frame.gyro_dps_x10[i];
        rehab_frame.wrist_angle_x100[i] = wrist_frame.angle_x100[i];
      }

      if (elbow_zero_valid == 0U)
      {
        elbow_zero_relative_pitch_x100 =
            Rehab_AngleDiffSignedX100(rehab_frame.wrist_angle_x100[1],
                                      rehab_frame.upper_angle_x100[1]);
        forearm_zero_roll_x100 = rehab_frame.wrist_angle_x100[0];
        arm_abd_zero_yaw_x100 = rehab_frame.upper_angle_x100[2];
        shoulder_zero_pitch_x100 = rehab_frame.upper_angle_x100[1];
        elbow_zero_valid = 1U;
        printf("REHAB posture zero calibrated elbow_rel=%ld.%02ld wrist_roll=%ld.%02ld upper_yaw=%ld.%02ld upper_pitch=%ld.%02ld deg\r\n",
               (long)(elbow_zero_relative_pitch_x100 / 100),
               (long)Rehab_AbsI32(elbow_zero_relative_pitch_x100 % 100),
               (long)(forearm_zero_roll_x100 / 100),
               (long)Rehab_AbsI32(forearm_zero_roll_x100 % 100),
               (long)(arm_abd_zero_yaw_x100 / 100),
               (long)Rehab_AbsI32(arm_abd_zero_yaw_x100 % 100),
               (long)(shoulder_zero_pitch_x100 / 100),
               (long)Rehab_AbsI32(shoulder_zero_pitch_x100 % 100));
      }

      rehab_frame.elbow_angle_x100 =
          Rehab_ComputeElbowFlexionX100(rehab_frame.upper_angle_x100[1],
                                        rehab_frame.wrist_angle_x100[1],
                                        elbow_zero_relative_pitch_x100,
                                        &rehab_frame.elbow_relative_pitch_x100);
      rehab_frame.forearm_rotation_angle_x100 =
          Rehab_AngleDiffAbsX100(rehab_frame.wrist_angle_x100[0], forearm_zero_roll_x100);
      rehab_frame.arm_abduction_angle_x100 =
          Rehab_AngleDiffAbsX100(rehab_frame.upper_angle_x100[2], arm_abd_zero_yaw_x100);
      rehab_frame.shoulder_lift_angle_x100 =
          Rehab_AngleDiffAbsX100(rehab_frame.upper_angle_x100[1], shoulder_zero_pitch_x100);

      if ((have_last_angle != 0U) && (now_tick > last_tick))
      {
        rehab_frame.elbow_velocity_x100_s =
            ((rehab_frame.elbow_angle_x100 - last_elbow_angle_x100) * 1000) /
            (int32_t)(now_tick - last_tick);
      }
      else
      {
        rehab_frame.elbow_velocity_x100_s = 0;
        have_last_angle = 1U;
      }

      last_elbow_angle_x100 = rehab_frame.elbow_angle_x100;
      last_tick = now_tick;

      if ((window_initialized == 0U) || ((now_tick - window_start_tick) >= REHAB_WINDOW_MS))
      {
        window_start_tick = now_tick;
        window_min_elbow_x100 = rehab_frame.elbow_angle_x100;
        window_max_elbow_x100 = rehab_frame.elbow_angle_x100;
        window_initialized = 1U;
      }
      else
      {
        if (rehab_frame.elbow_angle_x100 < window_min_elbow_x100)
        {
          window_min_elbow_x100 = rehab_frame.elbow_angle_x100;
        }
        if (rehab_frame.elbow_angle_x100 > window_max_elbow_x100)
        {
          window_max_elbow_x100 = rehab_frame.elbow_angle_x100;
        }
      }

      rehab_frame.elbow_range_x100 = window_max_elbow_x100 - window_min_elbow_x100;
      rehab_frame.emg_rms_x10 = emg_features.rms_x10;
      rehab_frame.emg_active = emg_features.active;
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
      forearm_upper_stable =
          ((Rehab_AbsI32(rehab_frame.upper_gyro_x10[1]) < REHAB_FOREARM_UPPER_STABLE_GYRO_X10) &&
           (Rehab_AbsI32(rehab_frame.upper_gyro_x10[2]) < REHAB_FOREARM_UPPER_STABLE_GYRO_X10) &&
           (rehab_frame.arm_abduction_angle_x100 < REHAB_FOREARM_UPPER_STABLE_ANGLE_X100) &&
           (rehab_frame.shoulder_lift_angle_x100 < REHAB_FOREARM_UPPER_STABLE_ANGLE_X100)) ? 1U : 0U;
#endif
      rule_action = Rehab_ClassifyRule(&rehab_frame, previous_rule_action);

      if (rule_action != REHAB_ACTION_IDLE)
      {
        latched_action = rule_action;
        latched_action_tick = now_tick;
      }
      else if ((latched_action != REHAB_ACTION_IDLE) &&
               ((now_tick - latched_action_tick) <= REHAB_ACTION_LATCH_MS))
      {
        rule_action = latched_action;
      }
      else
      {
        latched_action = REHAB_ACTION_IDLE;
      }
      previous_rule_action = rule_action;

      rehab_frame.action = rule_action;
      rehab_frame.level = (rehab_frame.action != REHAB_ACTION_IDLE) ? 1U : 0U;

      #if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
      forearm_eval_sample.rotation_angle_x100 = rehab_frame.forearm_rotation_angle_x100;
      forearm_eval_sample.wrist_roll_gyro_x10 = rehab_frame.wrist_gyro_x10[0];
      forearm_eval_sample.upper_stable =
          ((forearm_upper_stable != 0U) ||
           (rehab_frame.forearm_rotation_angle_x100 >= REHAB_FOREARM_ROT_STRONG_X100)) ? 1U : 0U;
      forearm_eval_sample.valid = rehab_frame.valid;
      forearm_eval_sample.tick_ms = now_tick;
      ForearmRotEval_Update(&forearm_eval_sample);
      ForearmRotEval_GetState(&forearm_eval_state);

      if ((forearm_eval_state.just_completed_rep != 0U) ||
          ((now_tick - last_train_print_tick) >= 500U))
      {
        last_train_print_tick = now_tick;
        printf("TRAIN FOREARM_ROT rep=%u/%u upper_roll=%c%ld.%02ld wrist_roll=%c%ld.%02ld rot=%ld.%02ld gyro=%c%ld.%01ld max=%ld.%02ld score=%u stable=%u\r\n",
               (unsigned int)forearm_eval_state.reps,
               (unsigned int)forearm_eval_state.target_reps,
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[0]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[0]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[0]) % 100),
               Rehab_SignCharI32(rehab_frame.wrist_angle_x100[0]),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[0]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[0]) % 100),
               (long)(forearm_eval_state.current_angle_x100 / 100),
               (long)Rehab_AbsI32(forearm_eval_state.current_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.wrist_gyro_x10[0]),
               (long)(Rehab_AbsI32(rehab_frame.wrist_gyro_x10[0]) / 10),
               (long)(Rehab_AbsI32(rehab_frame.wrist_gyro_x10[0]) % 10),
               (long)(forearm_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(forearm_eval_state.max_angle_x100 % 100),
               (unsigned int)forearm_eval_state.total_score,
               (unsigned int)forearm_upper_stable);
      }

      if (forearm_eval_state.just_finished != 0U)
      {
        printf("RESULT FOREARM_ROT reps=%u max=%ld.%02ld avg=%ld.%02ld score=%u\r\n",
               (unsigned int)forearm_eval_state.reps,
               (long)(forearm_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(forearm_eval_state.max_angle_x100 % 100),
               (long)(forearm_eval_state.avg_peak_angle_x100 / 100),
               (long)Rehab_AbsI32(forearm_eval_state.avg_peak_angle_x100 % 100),
               (unsigned int)forearm_eval_state.total_score);
      }
      #elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_ARM_ABD)
      arm_abd_eval_sample.abduction_angle_x100 = rehab_frame.arm_abduction_angle_x100;
      arm_abd_eval_sample.upper_yaw_gyro_x10 = rehab_frame.upper_gyro_x10[2];
      arm_abd_eval_sample.valid = rehab_frame.valid;
      arm_abd_eval_sample.tick_ms = now_tick;
      ArmAbdEval_Update(&arm_abd_eval_sample);
      ArmAbdEval_GetState(&arm_abd_eval_state);

      if ((arm_abd_eval_state.just_completed_rep != 0U) ||
          ((now_tick - last_train_print_tick) >= 500U))
      {
        last_train_print_tick = now_tick;
        printf("TRAIN ARM_ABD rep=%u/%u abd=%ld.%02ld upper_yaw=%c%ld.%02ld upper_pitch=%c%ld.%02ld gyro_z=%c%ld.%01ld max=%ld.%02ld score=%u\r\n",
               (unsigned int)arm_abd_eval_state.reps,
               (unsigned int)arm_abd_eval_state.target_reps,
               (long)(arm_abd_eval_state.current_angle_x100 / 100),
               (long)Rehab_AbsI32(arm_abd_eval_state.current_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[2]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[1]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) % 100),
               Rehab_SignCharI32(rehab_frame.upper_gyro_x10[2]),
               (long)(Rehab_AbsI32(rehab_frame.upper_gyro_x10[2]) / 10),
               (long)(Rehab_AbsI32(rehab_frame.upper_gyro_x10[2]) % 10),
               (long)(arm_abd_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(arm_abd_eval_state.max_angle_x100 % 100),
               (unsigned int)arm_abd_eval_state.total_score);
      }

      if (arm_abd_eval_state.just_finished != 0U)
      {
        printf("RESULT ARM_ABD reps=%u max=%ld.%02ld avg=%ld.%02ld score=%u\r\n",
               (unsigned int)arm_abd_eval_state.reps,
               (long)(arm_abd_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(arm_abd_eval_state.max_angle_x100 % 100),
               (long)(arm_abd_eval_state.avg_peak_angle_x100 / 100),
               (long)Rehab_AbsI32(arm_abd_eval_state.avg_peak_angle_x100 % 100),
               (unsigned int)arm_abd_eval_state.total_score);
      }
      #elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_SHOULDER_LIFT)
      shoulder_lift_eval_sample.lift_angle_x100 = rehab_frame.shoulder_lift_angle_x100;
      shoulder_lift_eval_sample.upper_pitch_gyro_x10 = rehab_frame.upper_gyro_x10[1];
      shoulder_lift_eval_sample.valid = rehab_frame.valid;
      shoulder_lift_eval_sample.tick_ms = now_tick;
      ShoulderLiftEval_Update(&shoulder_lift_eval_sample);
      ShoulderLiftEval_GetState(&shoulder_lift_eval_state);

      if ((shoulder_lift_eval_state.just_completed_rep != 0U) ||
          ((now_tick - last_train_print_tick) >= 500U))
      {
        last_train_print_tick = now_tick;
        printf("TRAIN SHOULDER_LIFT rep=%u/%u lift=%ld.%02ld upper_pitch=%c%ld.%02ld upper_yaw=%c%ld.%02ld gyro_y=%c%ld.%01ld max=%ld.%02ld score=%u\r\n",
               (unsigned int)shoulder_lift_eval_state.reps,
               (unsigned int)shoulder_lift_eval_state.target_reps,
               (long)(shoulder_lift_eval_state.current_angle_x100 / 100),
               (long)Rehab_AbsI32(shoulder_lift_eval_state.current_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[1]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[2]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) % 100),
               Rehab_SignCharI32(rehab_frame.upper_gyro_x10[1]),
               (long)(Rehab_AbsI32(rehab_frame.upper_gyro_x10[1]) / 10),
               (long)(Rehab_AbsI32(rehab_frame.upper_gyro_x10[1]) % 10),
               (long)(shoulder_lift_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(shoulder_lift_eval_state.max_angle_x100 % 100),
               (unsigned int)shoulder_lift_eval_state.total_score);
      }

      if (shoulder_lift_eval_state.just_finished != 0U)
      {
        printf("RESULT SHOULDER_LIFT reps=%u max=%ld.%02ld avg=%ld.%02ld score=%u\r\n",
               (unsigned int)shoulder_lift_eval_state.reps,
               (long)(shoulder_lift_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(shoulder_lift_eval_state.max_angle_x100 % 100),
               (long)(shoulder_lift_eval_state.avg_peak_angle_x100 / 100),
               (long)Rehab_AbsI32(shoulder_lift_eval_state.avg_peak_angle_x100 % 100),
               (unsigned int)shoulder_lift_eval_state.total_score);
      }
      #else
      elbow_eval_sample.flex_angle_x100 = rehab_frame.elbow_angle_x100;
      elbow_eval_sample.emg_active = rehab_frame.emg_active;
      elbow_eval_sample.valid = rehab_frame.valid;
      elbow_eval_sample.tick_ms = now_tick;
      ElbowFlexEval_Update(&elbow_eval_sample);
      ElbowFlexEval_GetState(&elbow_eval_state);

      if ((elbow_eval_state.just_completed_rep != 0U) ||
          ((now_tick - last_train_print_tick) >= 500U))
      {
        last_train_print_tick = now_tick;
        printf("TRAIN ELBOW_FLEX rep=%u/%u flex=%ld.%02ld upper_pitch=%c%ld.%02ld wrist_pitch=%c%ld.%02ld max=%ld.%02ld score=%u emg=%u%%\r\n",
               (unsigned int)elbow_eval_state.reps,
               (unsigned int)elbow_eval_state.target_reps,
               (long)(elbow_eval_state.current_angle_x100 / 100),
               (long)Rehab_AbsI32(elbow_eval_state.current_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[1]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) % 100),
               Rehab_SignCharI32(rehab_frame.wrist_angle_x100[1]),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[1]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[1]) % 100),
               (long)(elbow_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(elbow_eval_state.max_angle_x100 % 100),
               (unsigned int)elbow_eval_state.total_score,
               (unsigned int)elbow_eval_state.emg_rate);
      }

      if (elbow_eval_state.just_finished != 0U)
      {
        printf("RESULT ELBOW_FLEX reps=%u max=%ld.%02ld avg=%ld.%02ld score=%u emg=%u%%\r\n",
               (unsigned int)elbow_eval_state.reps,
               (long)(elbow_eval_state.max_angle_x100 / 100),
               (long)Rehab_AbsI32(elbow_eval_state.max_angle_x100 % 100),
               (long)(elbow_eval_state.avg_peak_angle_x100 / 100),
               (long)Rehab_AbsI32(elbow_eval_state.avg_peak_angle_x100 % 100),
               (unsigned int)elbow_eval_state.total_score,
               (unsigned int)elbow_eval_state.emg_rate);
      }
      #endif

      taskENTER_CRITICAL();
      latest_rehab_frame = rehab_frame;
      latest_rehab_valid = 1U;
      taskEXIT_CRITICAL();
    }
    else
    {
      have_last_angle = 0U;
      previous_rule_action = REHAB_ACTION_IDLE;
      latched_action = REHAB_ACTION_IDLE;
      rule_action = REHAB_ACTION_IDLE;
      #if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
      memset(&forearm_eval_sample, 0, sizeof(forearm_eval_sample));
      forearm_eval_sample.tick_ms = now_tick;
      ForearmRotEval_Update(&forearm_eval_sample);
      #elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_ARM_ABD)
      memset(&arm_abd_eval_sample, 0, sizeof(arm_abd_eval_sample));
      arm_abd_eval_sample.tick_ms = now_tick;
      ArmAbdEval_Update(&arm_abd_eval_sample);
      #elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_SHOULDER_LIFT)
      memset(&shoulder_lift_eval_sample, 0, sizeof(shoulder_lift_eval_sample));
      shoulder_lift_eval_sample.tick_ms = now_tick;
      ShoulderLiftEval_Update(&shoulder_lift_eval_sample);
      #else
      memset(&elbow_eval_sample, 0, sizeof(elbow_eval_sample));
      elbow_eval_sample.tick_ms = now_tick;
      ElbowFlexEval_Update(&elbow_eval_sample);
      #endif
    }

    if ((now_tick - last_print_tick) >= 200U)
    {
      last_print_tick = now_tick;
      if (rehab_frame.valid != 0U)
      {
#if (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_FOREARM_ROT)
        printf("REHAB action=%s rot=%ld.%02ld upper_roll=%c%ld.%02ld wrist_roll=%c%ld.%02ld emg=%u\r\n",
               Rehab_ActionName(rehab_frame.action),
               (long)(rehab_frame.forearm_rotation_angle_x100 / 100),
               (long)Rehab_AbsI32(rehab_frame.forearm_rotation_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[0]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[0]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[0]) % 100),
               Rehab_SignCharI32(rehab_frame.wrist_angle_x100[0]),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[0]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[0]) % 100),
               (unsigned int)rehab_frame.emg_active);
#elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_ARM_ABD)
        printf("REHAB action=%s abd=%ld.%02ld upper_yaw=%c%ld.%02ld upper_pitch=%c%ld.%02ld emg=%u\r\n",
               Rehab_ActionName(rehab_frame.action),
               (long)(rehab_frame.arm_abduction_angle_x100 / 100),
               (long)Rehab_AbsI32(rehab_frame.arm_abduction_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[2]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[1]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) % 100),
               (unsigned int)rehab_frame.emg_active);
#elif (REHAB_ACTIVE_EVAL == REHAB_ACTIVE_EVAL_SHOULDER_LIFT)
        printf("REHAB action=%s lift=%ld.%02ld upper_pitch=%c%ld.%02ld upper_yaw=%c%ld.%02ld emg=%u\r\n",
               Rehab_ActionName(rehab_frame.action),
               (long)(rehab_frame.shoulder_lift_angle_x100 / 100),
               (long)Rehab_AbsI32(rehab_frame.shoulder_lift_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[1]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[1]) % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[2]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[2]) % 100),
               (unsigned int)rehab_frame.emg_active);
#else
        printf("REHAB action=%s flex=%ld.%02ld upper_roll=%c%ld.%02ld wrist_roll=%c%ld.%02ld emg=%u\r\n",
               Rehab_ActionName(rehab_frame.action),
               (long)(rehab_frame.elbow_angle_x100 / 100),
               (long)Rehab_AbsI32(rehab_frame.elbow_angle_x100 % 100),
               Rehab_SignCharI32(rehab_frame.upper_angle_x100[0]),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[0]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.upper_angle_x100[0]) % 100),
               Rehab_SignCharI32(rehab_frame.wrist_angle_x100[0]),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[0]) / 100),
               (long)(Rehab_AbsI32(rehab_frame.wrist_angle_x100[0]) % 100),
               (unsigned int)rehab_frame.emg_active);
#endif
      }
      else
      {
        printf("REHAB waiting upper=%u wrist=%u wrist_status=0x%02X\r\n",
               (unsigned int)upper_valid,
               (unsigned int)wrist_valid,
               (unsigned int)wrist_frame.status);
      }
    }

    osDelay(REHAB_ALGO_PERIOD_MS);
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
// Poll USART1 bytes from the HC-05 module and parse wrist-node frames.
void StartWirelessTask(void *argument)
{
  /* USER CODE BEGIN WirelessTask */
  uint8_t rx_byte;
  WirelessWristFrame_t wrist_frame;
  WirelessLinkStats_t stats;
  uint8_t raw_sample[16];
  uint32_t last_debug_tick = 0U;
  uint32_t last_frame_tick = 0U;
  uint32_t raw_rx_count = 0U;
  uint32_t last_raw_rx_count = 0U;
  uint8_t raw_sample_count = 0U;
  uint8_t has_frame = 0U;

  (void)argument;

  WirelessLink_Init();
  printf("HC05 USART1 RX task start, baud=9600\r\n");

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(uart1MutexHandle, osWaitForever);
    if (HAL_UART_Receive(&huart1, &rx_byte, 1U, 20U) == HAL_OK)
    {
      osMutexRelease(uart1MutexHandle);
      raw_rx_count++;
      if (raw_sample_count < sizeof(raw_sample))
      {
        raw_sample[raw_sample_count++] = rx_byte;
      }

      if (WirelessLink_PushByte(rx_byte, &wrist_frame) != 0U)
      {
        taskENTER_CRITICAL();
        latest_wrist_frame = wrist_frame;
        latest_wrist_valid = 1U;
        taskEXIT_CRITICAL();

        has_frame = 1U;
        last_frame_tick = osKernelGetTickCount();
      }
    }
    else
    {
      osMutexRelease(uart1MutexHandle);
    }

    if ((osKernelGetTickCount() - last_debug_tick) >= 1000U)
    {
      last_debug_tick = osKernelGetTickCount();
      WirelessLink_GetStats(&stats);

      if (has_frame != 0U)
      {
        printf("HC05_RX ok=%lu err=%lu seq=%u status=0x%02X acc_mg=%d,%d,%d gyro_x10=%d,%d,%d angle_x100=%d,%d,%d age=%lu\r\n",
               (unsigned long)stats.frames_ok,
               (unsigned long)stats.checksum_errors,
               (unsigned int)wrist_frame.seq,
               (unsigned int)wrist_frame.status,
               (int)wrist_frame.acc_mg[0],
               (int)wrist_frame.acc_mg[1],
               (int)wrist_frame.acc_mg[2],
               (int)wrist_frame.gyro_dps_x10[0],
               (int)wrist_frame.gyro_dps_x10[1],
               (int)wrist_frame.gyro_dps_x10[2],
               (int)wrist_frame.angle_x100[0],
               (int)wrist_frame.angle_x100[1],
               (int)wrist_frame.angle_x100[2],
               (unsigned long)(osKernelGetTickCount() - last_frame_tick));
      }
      else
      {
        printf("HC05_RX waiting bytes=%lu delta=%lu resync=%lu err=%lu raw:",
               (unsigned long)raw_rx_count,
               (unsigned long)(raw_rx_count - last_raw_rx_count),
               (unsigned long)stats.resync_count,
               (unsigned long)stats.checksum_errors);
        for (uint8_t i = 0U; i < raw_sample_count; i++)
        {
          printf(" %02X", (unsigned int)raw_sample[i]);
        }
        printf("\r\n");
      }

      last_raw_rx_count = raw_rx_count;
      raw_sample_count = 0U;
    }

    osDelay(1);
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
  int len;

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

    len = snprintf(line,
                   sizeof(line),
                   "%u %ld %ld %ld %ld %ld\r\n",
                   (unsigned int)emg_features.raw,
                   (long)emg_features.baseline_x10,
                   (long)emg_features.drop_x10,
                   (long)emg_features.rectified_x10,
                   (long)emg_features.envelope_x10,
                   (long)emg_features.rms_x10);

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

/* USART1 收到一个腕端字节：喂给帧解析器后立即重新挂起下一字节接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    (void)WirelessLink_PushByte(wireless_rx_byte, NULL);
    (void)HAL_UART_Receive_IT(&huart1, &wireless_rx_byte, 1U);
  }
}

/* USER CODE END Application */

