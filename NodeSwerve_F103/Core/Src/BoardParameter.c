/*
 * BoardParameter.c
 *
 *  Created on: Sep 23, 2023
 *      Author: KHOA
 */
#include "BoardParameter.h"
#include "Motor.h"
#include "cmsis_os.h"
BoardParameter_t brdParam;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;



float brd_GetCurrentAngleDC(){return encoder_GetPulse(&brdParam.encDC, MODE_ANGLE);}
float brd_GetCurrentSpeedBLDC(){return encoder_GetSpeed(&brdParam.encBLDC);}
int brd_GetCurrentCountBLDC(){return (int)encoder_GetPulse(&brdParam.encBLDC, MODE_X4);}

void brd_Init()
{
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	encoder_Init(&brdParam.encDC, &htim3, DCEncoderPerRound*DCGearRatio, PIDDeltaT);
	encoder_Init(&brdParam.encBLDC, &htim4, _BLDCEncoderPerRound*_BLDCGearRatio, PIDDeltaT);
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
	brdParam.pidBLDC_Speed.kP = 0.1;
	brdParam.pidBLDC_Speed.kI = 5;
	brdParam.pidBLDC_Speed.kD = 0;
	brdParam.pidBLDC_Speed.alpha = 0;
	brdParam.pidBLDC_Speed.deltaT = PIDDeltaT;
	brdParam.pidBLDC_Speed.u_AboveLimit = 1000;
	brdParam.pidBLDC_Speed.u_BelowLimit = -1000;
	brdParam.pidBLDC_Speed.kB = 1/PIDDeltaT;

	brdParam.pidDC_Angle.kP = 5;
	brdParam.pidDC_Angle.kI = 0;
	brdParam.pidDC_Angle.kD = 0.04;
	brdParam.pidDC_Angle.alpha = 0.8;
	brdParam.pidDC_Angle.deltaT = PIDDeltaT;
	brdParam.pidDC_Angle.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidDC_Angle.u_BelowLimit = DC_SUM_BELOW_LIMIT;
	brdParam.pidDC_Angle.kB = 1/PIDDeltaT;

	brdParam.pidDC_Speed.kP = 1;
	brdParam.pidDC_Speed.kI = 200;
	brdParam.pidDC_Speed.kD = 0;
	brdParam.pidDC_Speed.alpha = 0;
	brdParam.pidDC_Speed.deltaT = PIDDeltaT;
	brdParam.pidDC_Speed.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidDC_Speed.u_BelowLimit = DC_SUM_BELOW_LIMIT;
	brdParam.pidDC_Speed.kB = 1/PIDDeltaT;
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

void brd_ResetState()
{
	brd_SetTargetAngleDC(0);
	brdParam.pidDC_Speed.u_AboveLimit = 0;
	brdParam.pidDC_Speed.u_BelowLimit= 0;
	encoder_ResetCount(&brdParam.encDC);
	osDelay(1000);
	brdParam.pidDC_Speed.uI = 0;
//	brdParam.pidDC_Speed.u_AboveLimit = 100;
//	brdParam.pidDC_Speed.u_BelowLimit= -100;
//	brdParam.pidDC_Speed.uI_AboveLimit = DC_INTERGRAL_ABOVE_LIMIT;
//	brdParam.pidDC_Speed.uI_BelowLimit = DC_INTERGRAL_BELOW_LIMIT;
	brdParam.pidDC_Speed.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidDC_Speed.u_BelowLimit= DC_SUM_BELOW_LIMIT;
}

void brd_SetHomeCompleteCallback()
{
	encoder_ResetCount(&brdParam.encDC);

}


float brd_GetTargetAngleDC(){return brdParam.targetAngleDC;}
void brd_SetTargetAngleDC(float angle){brdParam.targetAngleDC = angle;}

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

void brd_SetEncX4BLDC(int32_t countX4){brdParam.encBLDC.count_X4 = countX4;}
int32_t brd_GetEncX4BLDC(){return brdParam.encBLDC.count_X4;}

float brd_GetDeltaT(){return brdParam.pidDC_Speed.deltaT;}
void brd_SetDeltaT(float deltaT){
	brdParam.pidDC_Speed.deltaT =
	brdParam.pidDC_Angle.deltaT =
	brdParam.pidBLDC_Speed.deltaT = deltaT;
}
