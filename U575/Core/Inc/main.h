/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define DEBUG_TX_Pin GPIO_PIN_0
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_1
#define DEBUG_RX_GPIO_Port GPIOA
#define IMU_ARM_CS_Pin GPIO_PIN_4
#define IMU_ARM_CS_GPIO_Port GPIOA
#define IMU_ARM_SCK_Pin GPIO_PIN_5
#define IMU_ARM_SCK_GPIO_Port GPIOA
#define IMU_ARM_MISO_Pin GPIO_PIN_6
#define IMU_ARM_MISO_GPIO_Port GPIOA
#define IMU_ARM_MOSI_Pin GPIO_PIN_7
#define IMU_ARM_MOSI_GPIO_Port GPIOA
#define TP_INT_Pin GPIO_PIN_4
#define TP_INT_GPIO_Port GPIOC
#define TP_RST_Pin GPIO_PIN_5
#define TP_RST_GPIO_Port GPIOC
#define EMG_ADC_Pin GPIO_PIN_0
#define EMG_ADC_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_6
#define LED_STATUS_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_8
#define LCD_BL_GPIO_Port GPIOA
#define WIRELESS_RXD_Pin GPIO_PIN_9
#define WIRELESS_RXD_GPIO_Port GPIOA
#define WIRELESS_TXD_Pin GPIO_PIN_10
#define WIRELESS_TXD_GPIO_Port GPIOA
#define USER_KEY_Pin GPIO_PIN_12
#define USER_KEY_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOC
#define LCD_SDA_Pin GPIO_PIN_12
#define LCD_SDA_GPIO_Port GPIOC
#define TP_SCL_Pin GPIO_PIN_6
#define TP_SCL_GPIO_Port GPIOB
#define TP_SDA_Pin GPIO_PIN_7
#define TP_SDA_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_8
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_9
#define LCD_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
