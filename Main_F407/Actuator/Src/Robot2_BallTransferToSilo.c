/*
 * Robot2_BallTransferToSilo.c
 *
 *  Created on: Mar 20, 2024
 *      Author: SpiritBoi
 */

#include "Robot2_BallTransferToSilo.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

#define DELTA_T 0.001

RuloParam_t rulo;
ForkliftParam_t forklift;

void balltransfer_Init() {
	MotorDC_Init(&forklift.mdc, &htim3, MOTOR_PWM_INVERSE, TIM_CHANNEL_1, TIM_CHANNEL_2);
	MotorDC_Init(&rulo.mdc, &htim5, MOTOR_PWM_NORMAL, TIM_CHANNEL_1, TIM_CHANNEL_2);
	encoder_Init(&forklift.enc, &htim2, 19200, DELTA_T);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void balltransfer_Rulo_Activate(bool activate) {
	if(activate)
		MotorDC_Drive(&rulo.mdc, -800);
	else
		MotorDC_Drive(&rulo.mdc, 0);
}

void balltransfer_Forklift_Move(float speed) {
	MotorDC_Drive(&forklift.mdc, speed);
}

int32_t balltransfer_Forklift_GetEncoderPulse() {
	return encoder_GetPulse(&forklift.enc, MODE_X4);
}
