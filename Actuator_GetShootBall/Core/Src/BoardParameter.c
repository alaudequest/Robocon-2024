/*
 * BoardParameter.c
 *
 *  Created on: Jan 18, 2024
 *      Author: NamDHay
 */

#include "BoardParameter.h"

BoardParameter_t brdParam;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void brd_Init()
{
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	Motor_Init(&brdParam.rotary, MOTOR_PWM, 0, 0, &htim1, TIM_CHANNEL_2, TIM_CHANNEL_3);
	Motor_Init(&brdParam.gun1, MOTOR_PWM, 0, 0, &htim2, TIM_CHANNEL_3, 0);
	Motor_Init(&brdParam.gun2, MOTOR_PWM, 0, 0, &htim2, TIM_CHANNEL_4, 0);
	Motor_Init(&brdParam.ball1, MOTOR_LL, RuloBall1_GPIO_Port, RuloBall1_Pin, NULL, 0, 0);
	Motor_Init(&brdParam.ball2, MOTOR_LL, RuloBall2_GPIO_Port, RuloBall2_Pin, NULL, 0, 0);

	encoder_Init_InterruptMode(&brdParam.encGun1, _Gun1EncoderPerRound*_Gun1GearRatio, _Gun1DeltaT);
	encoder_Init_InterruptMode(&brdParam.encGun2, _Gun2EncoderPerRound*_Gun2GearRatio, _Gun2DeltaT);
	encoder_Init_EncoderMode(&brdParam.encRotary, &htim3, RotaryEncoderPerRound*RotaryGearRatio, RotaryDeltaT);

	brdParam.pidRotaryAngle.kP = 0.1;
	brdParam.pidRotaryAngle.kI = 5;
	brdParam.pidRotaryAngle.kD = 0;
	brdParam.pidRotaryAngle.alpha = 0;
	brdParam.pidRotaryAngle.deltaT = PIDDeltaT;
	brdParam.pidRotaryAngle.u_AboveLimit = 1000;
	brdParam.pidRotaryAngle.u_BelowLimit = -1000;
	brdParam.pidRotaryAngle.kB = 1/PIDDeltaT;

	brdParam.pidRotarySpeed.kP = 0.1;
	brdParam.pidRotarySpeed.kI = 5;
	brdParam.pidRotarySpeed.kD = 0;
	brdParam.pidRotarySpeed.alpha = 0;
	brdParam.pidRotarySpeed.deltaT = PIDDeltaT;
	brdParam.pidRotarySpeed.u_AboveLimit = 1000;
	brdParam.pidRotarySpeed.u_BelowLimit = -1000;
	brdParam.pidRotarySpeed.kB = 1/PIDDeltaT;

	brdParam.pidGun2.kP = 5;
	brdParam.pidGun2.kI = 0;
	brdParam.pidGun2.kD = 0.04;
	brdParam.pidGun2.alpha = 0.8;
	brdParam.pidGun2.deltaT = PIDDeltaT;
	brdParam.pidGun2.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidGun2.u_BelowLimit = DC_SUM_BELOW_LIMIT;
	brdParam.pidGun2.kB = 1/PIDDeltaT;

	brdParam.pidGun1.kP = 1;
	brdParam.pidGun1.kI = 200;
	brdParam.pidGun1.kD = 0;
	brdParam.pidGun1.alpha = 0;
	brdParam.pidGun1.deltaT = PIDDeltaT;
	brdParam.pidGun1.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidGun1.u_BelowLimit = DC_SUM_BELOW_LIMIT;
	brdParam.pidGun1.kB = 1/PIDDeltaT;
}

void brd_SetHomeCompleteCallback(){
	encoder_ResetCount(&brdParam.encRotary);
}

void brd_SetPID(PID_Param pid,PID_type type){
	switch(type){
	case PID_GUN1:
		brdParam.pidGun1 = pid;
		break;
	case PID_GUN2:
		brdParam.pidGun2 = pid;
		break;
	case PID_ROTARY_ANGLE:
		brdParam.pidRotaryAngle = pid;
		break;
	case PID_ROTARY_SPEED:
		brdParam.pidRotarySpeed = pid;
		break;
	default:
		break;
	}
}

PID_Param brd_GetPID(PID_type type){
	switch(type){
	case PID_GUN1:
		return brdParam.pidGun1;
		break;
	case PID_GUN2:
		return brdParam.pidGun2;
		break;
	case PID_ROTARY_ANGLE:
		return brdParam.pidRotaryAngle;
		break;
	case PID_ROTARY_SPEED:
		return brdParam.pidRotarySpeed;
		break;
	default:
		break;
	}
	return brdParam.pidRotarySpeed;
}

void brd_SetObjEncRotary(Encoder_t encRotary){brdParam.encRotary = encRotary;}
Encoder_t brd_GetObjEncRotary(){return brdParam.encRotary;}

void brd_SetObjEncGun(Encoder_t encGun, Motor_Type gun){
	if(gun == MOTOR_GUN1)	brdParam.encGun1 = encGun;
	else					brdParam.encGun2 = encGun;

}
Encoder_t brd_GetObjEncGun(Motor_Type gun){
	if(gun == MOTOR_GUN1)	return brdParam.encGun1;
	else					return brdParam.encGun2;
}

Motor brd_GetObjMotor(Motor_Type type){
	switch(type){
	case MOTOR_GUN1:
		return brdParam.gun1;
		break;
	case MOTOR_GUN2:
		return brdParam.gun2;
		break;
	case MOTOR_BALL1:
		return brdParam.ball1;
		break;
	case MOTOR_BALL2:
		return brdParam.ball2;
		break;
	case MOTOR_ROTARY:
		return brdParam.rotary;
		break;
	}
	return brdParam.rotary;
}

void brd_SetObjMotor(Motor motor, Motor_Type type){
	switch(type){
	case MOTOR_GUN1:
		brdParam.gun1 = motor;
		break;
	case MOTOR_GUN2:
		brdParam.gun2 = motor;
		break;
	case MOTOR_BALL1:
		brdParam.ball1 = motor;
		break;
	case MOTOR_BALL2:
		brdParam.ball2 = motor;
		break;
	case MOTOR_ROTARY:
		brdParam.rotary = motor;
		break;
	}
}

float brd_GetTargetRotaryAngle(){return brdParam.targetAngleDC;}
void brd_SetTargetRotaryAngle(float angle){brdParam.targetAngleDC = angle;}

float brd_GetSpeedGun(Motor_Type type){
	if(type == MOTOR_GUN1)	return brdParam.targetSpeedGun1;
	else 					return brdParam.targetSpeedGun2;
}
void brd_SetSpeedGun(float speed, Motor_Type type){
	if(type == MOTOR_GUN1)	brdParam.targetSpeedGun1 = speed;
	else					brdParam.targetSpeedGun2 = speed;
}

float brd_GetDeltaT(){return brdParam.pidGun1.deltaT;}
void brd_SetDeltaT(float deltaT){
	brdParam.pidGun1.deltaT = deltaT;
	brdParam.pidGun2.deltaT = deltaT;
	brdParam.pidRotaryAngle.deltaT = deltaT;
	brdParam.pidRotarySpeed.deltaT = deltaT;
}

void brd_ResetState(){
	brd_SetTargetRotaryAngle(0);
	encoder_ResetCount(&brdParam.encGun1);
	encoder_ResetCount(&brdParam.encGun2);
}
