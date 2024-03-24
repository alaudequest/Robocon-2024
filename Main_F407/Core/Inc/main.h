/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LibraryConfig.h"
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
#define HC595_CLK_Pin GPIO_PIN_4
#define HC595_CLK_GPIO_Port GPIOA
#define HC595_RCLK_Pin GPIO_PIN_5
#define HC595_RCLK_GPIO_Port GPIOA
#define HC595_OE_Pin GPIO_PIN_6
#define HC595_OE_GPIO_Port GPIOA
#define HC595_DATA_Pin GPIO_PIN_7
#define HC595_DATA_GPIO_Port GPIOA
#define RB1SensorPushBallUp_Pin GPIO_PIN_12
#define RB1SensorPushBallUp_GPIO_Port GPIOE
#define RB1SensorArmRight_Pin GPIO_PIN_13
#define RB1SensorArmRight_GPIO_Port GPIOE
#define RB1SensorArmRight_EXTI_IRQn EXTI15_10_IRQn
#define RB1SensorArmLeft_Pin GPIO_PIN_14
#define RB1SensorArmLeft_GPIO_Port GPIOE
#define RB1SensorArmLeft_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
