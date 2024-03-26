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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

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
#define sensor_1_Pin GPIO_PIN_7
#define sensor_1_GPIO_Port GPIOE
#define sensor_2_Pin GPIO_PIN_8
#define sensor_2_GPIO_Port GPIOE
#define sensor_3_Pin GPIO_PIN_9
#define sensor_3_GPIO_Port GPIOE
#define sensor_4_Pin GPIO_PIN_10
#define sensor_4_GPIO_Port GPIOE
#define sensor_5_Pin GPIO_PIN_11
#define sensor_5_GPIO_Port GPIOE
#define sensor_6_Pin GPIO_PIN_12
#define sensor_6_GPIO_Port GPIOE
#define sensor_7_Pin GPIO_PIN_13
#define sensor_7_GPIO_Port GPIOE
#define sensor_8_Pin GPIO_PIN_14
#define sensor_8_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
