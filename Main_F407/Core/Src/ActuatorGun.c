	/*
 * ActuatorGun.c
 *
 *  Created on: Mar 24, 2024
 *      Author: namdhay
 */


#include "ActuatorGun.h"
extern TIM_HandleTypeDef htim5;
void gun_Init() {
	// Start MOTOR GUN INIT
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	// End MOTOR GUN INIT

	// Start MOTOR RULO GET BALL INIT

	// End MOTOR RULO GET BALL INIT
}

void gun_StartGetBall() {
	HAL_GPIO_WritePin(MotorGetB1_GPIO_Port, MotorGetB1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorGetB2_GPIO_Port, MotorGetB2_Pin, GPIO_PIN_SET);
}
void gun_StopGetBall(){
	HAL_GPIO_WritePin(MotorGetB1_GPIO_Port, MotorGetB1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MotorGetB2_GPIO_Port, MotorGetB2_Pin, GPIO_PIN_RESET);
}

void gun_StartShootBall(uint16_t pwm) {
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm);
}
void gun_StopShootBall() {
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
}

void gun_StopAll() {
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(MotorGetB1_GPIO_Port, MotorGetB1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MotorGetB2_GPIO_Port, MotorGetB2_Pin, GPIO_PIN_RESET);
}
