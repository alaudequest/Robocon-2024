/*
 * OdometerHandle.c
 *
 *  Created on: Oct 25, 2023
 *      Author: Admin
 */
#include "OdometerHandle.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

odometerParam	odoParam;
void OdoInit(void){
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

	encoder_Init(&odoParam.encYR, &htim1, ODO_PULSE_PER_ROUND, ODO_DeltaT);
	encoder_Init(&odoParam.encYL, &htim3, ODO_PULSE_PER_ROUND, ODO_DeltaT);
	encoder_Init(&odoParam.encX , &htim2, ODO_PULSE_PER_ROUND, ODO_DeltaT);


	odoParam.EncCoef = 2*M_PI*ODO_WHEEL_RADIUS*1/(ODO_PULSE_PER_ROUND*4);
}

void Odo_PositionCalculate(void){

	float DeltaYR = encoder_GetPulse(&odoParam.encYR,MODE_X4);
	float DeltaYL = encoder_GetPulse(&odoParam.encYL,MODE_X4);
	float DeltaX  = encoder_GetPulse(&odoParam.encX ,MODE_X4);

	odoParam.DeltaCurrentYPos = odoParam.EncCoef*(DeltaYR + DeltaYL)/2;
	odoParam.DeltaCurrentTheta = odoParam.EncCoef*(DeltaYR - DeltaYL)/ODO_WHEEL_LR_DISTANCE;
	odoParam.DeltaCurrentXPos = odoParam.EncCoef*(DeltaX - ODO_WHEEL_UD_DISTANCE*(DeltaYR - DeltaYL)/ODO_WHEEL_LR_DISTANCE);

	encoder_ResetCount(&odoParam.encYR);
	encoder_ResetCount(&odoParam.encYL);
	encoder_ResetCount(&odoParam.encX);

	odoParam.CurrentXPos += odoParam.DeltaCurrentXPos*cos(odoParam.CurrentTheta*M_PI/180)-odoParam.DeltaCurrentYPos*sin(odoParam.CurrentTheta*M_PI/180);
	odoParam.CurrentYPos += odoParam.DeltaCurrentXPos*sin(odoParam.CurrentTheta*M_PI/180)+odoParam.DeltaCurrentYPos*cos(odoParam.CurrentTheta*M_PI/180);
	odoParam.CurrentTheta += odoParam.DeltaCurrentTheta;

	odoParam.DeltaCurrentYPos = 0;
	odoParam.DeltaCurrentTheta = 0;
	odoParam.DeltaCurrentXPos = 0;
}

int Odo_GetPulseYR(){return encoder_GetPulse(&odoParam.encYR,MODE_X4);}
int Odo_GetPulseYL(){return encoder_GetPulse(&odoParam.encYL,MODE_X4);}
int Odo_GetPulseX(){return encoder_GetPulse(&odoParam.encX,MODE_X4);}
