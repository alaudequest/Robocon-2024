/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define RuloBall1_Pin GPIO_PIN_0
#define RuloBall1_GPIO_Port GPIOA
#define RotaryGun1_Pin GPIO_PIN_2
#define RotaryGun1_GPIO_Port GPIOA
#define RotaryGun2_Pin GPIO_PIN_3
#define RotaryGun2_GPIO_Port GPIOA
#define RuloBall2_Pin GPIO_PIN_4
#define RuloBall2_GPIO_Port GPIOA
#define ECD2A_Pin GPIO_PIN_6
#define ECD2A_GPIO_Port GPIOA
#define ECD2B_Pin GPIO_PIN_7
#define ECD2B_GPIO_Port GPIOA
#define RuloGun2_Pin GPIO_PIN_9
#define RuloGun2_GPIO_Port GPIOA
#define RuloGun1_Pin GPIO_PIN_10
#define RuloGun1_GPIO_Port GPIOA
#define ECD3B_Pin GPIO_PIN_15
#define ECD3B_GPIO_Port GPIOA
#define ECD3B_EXTI_IRQn EXTI15_10_IRQn
#define ECD3A_Pin GPIO_PIN_3
#define ECD3A_GPIO_Port GPIOB
#define ECD3A_EXTI_IRQn EXTI3_IRQn
#define ECD1B_Pin GPIO_PIN_6
#define ECD1B_GPIO_Port GPIOB
#define ECD1B_EXTI_IRQn EXTI9_5_IRQn
#define ECD1A_Pin GPIO_PIN_7
#define ECD1A_GPIO_Port GPIOB
#define ECD1A_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */