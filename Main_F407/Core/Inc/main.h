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
#define BOARD_MAINF4_ROBOT2
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void process_Ball_Approach3(uint8_t Ball);
void process_setVal_PutBall(uint8_t value);
void process_PD_Critical();
void process_Accel_FloatingEnc3(float Angle, float maxSpeed, float s, float accel, float TargetAngle, float RotateTime, float AngleChange);
void process_Accel_FloatingEnc4(float Angle, float maxSpeed, float s, float accel, float TargetAngle, float RotateTime);
void Reset_MPU_Angle();
void process_PD_OnStrainghtPath();
void process_getBall();
void process_ResetFloatingEnc();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOE
#define Status_Pin GPIO_PIN_13
#define Status_GPIO_Port GPIOC
#define RobotSignalBtn_VCC_Pin GPIO_PIN_2
#define RobotSignalBtn_VCC_GPIO_Port GPIOC
#define RobotSignalBtn_GND_Pin GPIO_PIN_3
#define RobotSignalBtn_GND_GPIO_Port GPIOC
#define HC595_CLK_Pin GPIO_PIN_4
#define HC595_CLK_GPIO_Port GPIOA
#define HC595_RCLK_Pin GPIO_PIN_5
#define HC595_RCLK_GPIO_Port GPIOA
#define HC595_OE_Pin GPIO_PIN_6
#define HC595_OE_GPIO_Port GPIOA
#define HC595_DATA_Pin GPIO_PIN_7
#define HC595_DATA_GPIO_Port GPIOA
#define RobotSignalBtn_RED_Pin GPIO_PIN_4
#define RobotSignalBtn_RED_GPIO_Port GPIOC
#define RobotSignalBtn_YELLOW_Pin GPIO_PIN_5
#define RobotSignalBtn_YELLOW_GPIO_Port GPIOC
#define RobotSignalBtn_BLUE_Pin GPIO_PIN_0
#define RobotSignalBtn_BLUE_GPIO_Port GPIOB
#define RobotSignalBtn_GREEN_Pin GPIO_PIN_1
#define RobotSignalBtn_GREEN_GPIO_Port GPIOB
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
