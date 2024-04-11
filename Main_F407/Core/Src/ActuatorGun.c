/*
 * ActuatorGun.c
 *
 *  Created on: Mar 24, 2024
 *      Author: namdhay
 */

#include "ActuatorGun.h"
#include "PID.h"
#include "Encoder.h"
extern TIM_HandleTypeDef htim5;
Encoder_t ENC_Gun1;
Encoder_t ENC_Gun2;
PID_Param PID_Gun1;
PID_Param PID_Gun2;

void gun_PIDSetParam(PID_Param *pid, float kP, float kI, float kD, float alpha, float deltaT, float u_AboveLimit, float u_BelowLimit)
{
//----------------------Term-----------------------//
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->alpha = alpha;
	pid->kB = 1 / deltaT;
//----------------------Sample Time----------------//
	pid->deltaT = deltaT;
//----------------------Limit----------------------//
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}

void gun_Init() {
	// Start MOTOR GUN INIT
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	gun_PIDSetParam(&PID_Gun1, Gun1Proportion, Gun1Integral, Gun1Derivatite, Gun1Alpha, Gun1DeltaT, Gun1SumAboveLimit, Gun1SumBelowLimit);
	gun_PIDSetParam(&PID_Gun2, Gun2Proportion, Gun2Integral, Gun2Derivatite, Gun2Alpha, Gun2DeltaT, Gun2SumAboveLimit, Gun2SumBelowLimit);
	// End MOTOR GUN INIT

	// Start MOTOR RULO GET BALL INIT

	// End MOTOR RULO GET BALL INIT
}

void gun_StartGetBall() {
	HAL_GPIO_WritePin(MotorGetB1_GPIO_Port, MotorGetB1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorGetB2_GPIO_Port, MotorGetB2_Pin, GPIO_PIN_SET);
}
void gun_StopGetBall() {
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

void gun_VelCal(int gunCount1, int gunCount2) {
	VelCal(&ENC_Gun1, gunCount1, DCEncoderPerRound, Gun1DeltaT);
	VelCal(&ENC_Gun2, gunCount2, DCEncoderPerRound, Gun2DeltaT);
}

void gun_PIDSpeed1(float Target1) {
	PID_Calculate(&PID_Gun1, Target1, ENC_Gun1.vel_Real);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PID_Gun1.uHat);
}

void gun_PIDSpeed2(float Target2) {
	PID_Calculate(&PID_Gun2, Target2, ENC_Gun2.vel_Real);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PID_Gun2.uHat);
}
