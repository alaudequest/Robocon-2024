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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DIR_Pin GPIO_PIN_0
#define DIR_GPIO_Port GPIOA
#define BLDC_PWM_Pin GPIO_PIN_1
#define BLDC_PWM_GPIO_Port GPIOA
#define DC_PWMR_Pin GPIO_PIN_2
#define DC_PWMR_GPIO_Port GPIOA
#define DC_PWML_Pin GPIO_PIN_3
#define DC_PWML_GPIO_Port GPIOA
#define BRAKE_Pin GPIO_PIN_4
#define BRAKE_GPIO_Port GPIOA
#define Sensor_Pin GPIO_PIN_5
#define Sensor_GPIO_Port GPIOA
#define Sensor_EXTI_IRQn EXTI9_5_IRQn
#define EnA2_Pin GPIO_PIN_6
#define EnA2_GPIO_Port GPIOA
#define EnB2_Pin GPIO_PIN_7
#define EnB2_GPIO_Port GPIOA
#define EnB1_Pin GPIO_PIN_6
#define EnB1_GPIO_Port GPIOB
#define EnA1_Pin GPIO_PIN_7
#define EnA1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
