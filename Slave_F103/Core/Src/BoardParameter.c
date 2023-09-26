/*
 * BoardParameter.c
 *
 *  Created on: Sep 23, 2023
 *      Author: KHOA
 */
#include "BoardParameter.h"
#include "Motor.h"
BoardParameter_t brdParam;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;



void brd_Init()
{
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	encoder_Init(&brdParam.encDC, &htim3, DCEncoderPerRound, DCDeltaT);
	encoder_Init(&brdParam.encBLDC, &htim4, _BLDCEncoderPerRound, _BLDCDeltaT);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	MotorBLDC_Init(&brdParam.mbldc, &htim2, TIM_CHANNEL_2,
			BLDC_BRAKE_GPIO_Port,
			BLDC_BRAKE_Pin,
			BLDC_DIR_GPIO_Port,
			BLDC_DIR_Pin);
	MotorDC_Init(&brdParam.mdc, &htim2, MOTOR_PWM_INVERSE,
			TIM_CHANNEL_3,
			TIM_CHANNEL_4);
}

void brd_SetPID(PID_Param pid,PID_type type)
{
	switch(type){
	case PID_DC_SPEED:
		brdParam.pidDC_Speed = pid;
		break;
	case PID_DC_ANGLE:
		brdParam.pidDC_Angle = pid;
		break;
	case PID_BLDC_SPEED:
		brdParam.pidBLDC_Speed = pid;
		break;
	}
}

PID_Param brd_GetPID(PID_type type)
{
	switch(type){
	case PID_DC_SPEED:
		return brdParam.pidDC_Speed;
		break;
	case PID_DC_ANGLE:
		return brdParam.pidDC_Angle;
		break;
	case PID_BLDC_SPEED:
		return brdParam.pidBLDC_Speed;
		break;
	}
	return brdParam.pidDC_Speed;
}


MotorDC brd_GetObjMotorDC(){return brdParam.mdc;}
void brd_SetObjMotorDC(MotorDC mdc){brdParam.mdc = mdc;}

MotorBLDC brd_GetObjMotorBLDC(){return brdParam.mbldc;}
void brd_SetObjMotorBLDC(MotorBLDC mbldc){brdParam.mbldc = mbldc;}

void brd_SetSpeedBLDC(float speed){brdParam.targetSpeedBLDC = speed;}
float brd_GetSpeedBLDC(){return brdParam.targetSpeedBLDC;}

void brd_SetObjEncDC(Encoder_t encDC){brdParam.encDC = encDC;}
Encoder_t brd_GetObjEncDC(){return brdParam.encDC;}

void brd_SetObjEncBLDC(Encoder_t encBLDC){brdParam.encBLDC = encBLDC;}
Encoder_t brd_GetObjEncBLDC(){return brdParam.encBLDC;}

void brd_SetAngleDC(float angle){brdParam.targetAngleDC = angle;}
float brd_GetAngleDC(){return brdParam.targetAngleDC;}

void brd_SetEncX4BLDC(int32_t countX4){brdParam.encBLDC.count_X4 = countX4;}
int32_t brd_GetEncX4BLDC(){return brdParam.encBLDC.count_X4;}


