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
#define BOARD_MAINF4_ROBOT1
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOE
#define MotorGetB1_Pin GPIO_PIN_5
#define MotorGetB1_GPIO_Port GPIOE
#define MotorGetB2_Pin GPIO_PIN_6
#define MotorGetB2_GPIO_Port GPIOE
#define Status_Pin GPIO_PIN_13
#define Status_GPIO_Port GPIOC
#define RobotSignalBtn_VCC_Pin GPIO_PIN_2
#define RobotSignalBtn_VCC_GPIO_Port GPIOC
#define RobotSignalBtn_GND_Pin GPIO_PIN_3
#define RobotSignalBtn_GND_GPIO_Port GPIOC
#define MotorGun1_Pin GPIO_PIN_0
#define MotorGun1_GPIO_Port GPIOA
#define MotorGun2_Pin GPIO_PIN_1
#define MotorGun2_GPIO_Port GPIOA
#define HC595_CLK_Pin GPIO_PIN_4
#define HC595_CLK_GPIO_Port GPIOA
#define HC595_RCLK_Pin GPIO_PIN_5
#define HC595_RCLK_GPIO_Port GPIOA
#define HC595_OE_Pin GPIO_PIN_6
#define HC595_OE_GPIO_Port GPIOA
#define HC595_DATA_Pin GPIO_PIN_7
#define HC595_DATA_GPIO_Port GPIOA
#define Sensor1_Pin GPIO_PIN_7
#define Sensor1_GPIO_Port GPIOE
#define Sensor1_EXTI_IRQn EXTI9_5_IRQn
#define Sensor2_Pin GPIO_PIN_8
#define Sensor2_GPIO_Port GPIOE
#define Sensor2_EXTI_IRQn EXTI9_5_IRQn
#define Sensor4_Pin GPIO_PIN_10
#define Sensor4_GPIO_Port GPIOE
#define Sensor4_EXTI_IRQn EXTI15_10_IRQn
#define Sensor5_Pin GPIO_PIN_11
#define Sensor5_GPIO_Port GPIOE
#define Sensor5_EXTI_IRQn EXTI15_10_IRQn
#define Sensor6_Pin GPIO_PIN_12
#define Sensor6_GPIO_Port GPIOE
#define Sensor6_EXTI_IRQn EXTI15_10_IRQn
#define Sensor7_Pin GPIO_PIN_13
#define Sensor7_GPIO_Port GPIOE
#define Sensor7_EXTI_IRQn EXTI15_10_IRQn
#define Sensor8_Pin GPIO_PIN_14
#define Sensor8_GPIO_Port GPIOE
#define Sensor8_EXTI_IRQn EXTI15_10_IRQn
#define RobotSignalBtn_RED_Pin GPIO_PIN_8
#define RobotSignalBtn_RED_GPIO_Port GPIOD
#define RobotSignalBtn_YELLOW_Pin GPIO_PIN_9
#define RobotSignalBtn_YELLOW_GPIO_Port GPIOD
#define RobotSignalBtn_BLUE_Pin GPIO_PIN_10
#define RobotSignalBtn_BLUE_GPIO_Port GPIOD
#define RobotSignalBtn_GREEN_Pin GPIO_PIN_11
#define RobotSignalBtn_GREEN_GPIO_Port GPIOD
#define Enc2B_Pin GPIO_PIN_14
#define Enc2B_GPIO_Port GPIOD
#define Enc2A_Pin GPIO_PIN_15
#define Enc2A_GPIO_Port GPIOD
#define Enc2A_EXTI_IRQn EXTI15_10_IRQn
#define Enc1B_Pin GPIO_PIN_15
#define Enc1B_GPIO_Port GPIOA
#define Enc1A_Pin GPIO_PIN_3
#define Enc1A_GPIO_Port GPIOB
#define Enc1A_EXTI_IRQn EXTI3_IRQn
#define RelayRulo_Pin GPIO_PIN_8
#define RelayRulo_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
