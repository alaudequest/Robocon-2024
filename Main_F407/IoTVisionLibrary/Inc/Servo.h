/*
 * Servo.h
 *
 *  Created on: Mar 31, 2023
 *      Author: KHOA
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include "main.h"
#ifdef CONFIG_USE_SERVO
#define SERVO_LOW_LIMIT_DEG 50
#define SERVO_HIGH_LIMIT_DEG 130
#define SERVO_MAX_DUTY_CYCLE 2500
#define SERVO_MIN_DUTY_CYCLE 500
#define SERVO_START HAL_TIM_PWM_Start(svTemp->htim, svTemp->TIM_CHANNEL);
#define CONVERT_DEG_TO_PULSE(X) (((X)*SERVO_MAX_DUTY_CYCLE)/180)
#define SERVO_ANGLE_VALID(X) (((X) < SERVO_HIGH_LIMIT_DEG) && ((X) > SERVO_LOW_LIMIT_DEG) ? 1 : 0)
#define SERVO_TIMER svTemp->htim
#define SERVO_OUTPUT_CHANNEL svTemp->TIM_CHANNEL
#define SERVO_TURN(X) (__HAL_TIM_SET_COMPARE(SERVO_TIMER,SERVO_OUTPUT_CHANNEL,CONVERT_DEG_TO_PULSE((X))))

typedef struct Servo{
	TIM_HandleTypeDef *htim;
	uint32_t TIM_CHANNEL;
}Servo;

HAL_StatusTypeDef Servo_Config(Servo *sv, TIM_HandleTypeDef *htim, uint32_t TIM_CHANNEL);
HAL_StatusTypeDef Servo_SetAngle(uint8_t Angle);
HAL_StatusTypeDef Servo_SetTarget(Servo *sv);
HAL_StatusTypeDef Servo_Start();
#endif /* INC_SERVO_H_ */
#endif /* CONFIG_USE_SERVO */
