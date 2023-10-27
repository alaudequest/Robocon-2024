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
	encoder_Init(&odoParam.encYL, &htim2, ODO_PULSE_PER_ROUND, ODO_DeltaT);
	encoder_Init(&odoParam.encX , &htim3, ODO_PULSE_PER_ROUND, ODO_DeltaT);


	odoParam.EncCoef = 2*M_PI*ODO_WHEEL_RADIUS*1/(ODO_PULSE_PER_ROUND*4);
}

void PositionCalculate(void){

	float DeltaYR = encoder_GetPulse(&odoParam.encYR,MODE_X4);
	float DeltaYL = encoder_GetPulse(&odoParam.encYL,MODE_X4);
	float DeltaX  = encoder_GetPulse(&odoParam.encX ,MODE_X4);

	odoParam.DeltaCurrentYPos = odoParam.EncCoef*(DeltaYR + DeltaYL)/2;
<<<<<<< HEAD
	odoParam.DeltaCurrentTheta = odoParam.EncCoef*(DeltaYR - DeltaYL)/ODO_WHEEL_LR_DISTANCE;
	odoParam.DeltaCurrentXPos = odoParam.EncCoef*(DeltaX - ODO_WHEEL_UD_DISTANCE*(DeltaYR - DeltaYL)/ODO_WHEEL_LR_DISTANCE);
=======
	odoParam.DeltaCurrentTheta = odoParam.EncCoef*(DeltaYR - DeltaYL)/2;
//	odoParam.DeltaCurrentXPos = odoParam.EncCoef*(DeltaX - )
>>>>>>> origin/dev

	odoParam.CurrentXPos += odoParam.DeltaCurrentXPos*cos(odoParam.CurrentTheta)-odoParam.DeltaCurrentYPos*sin(odoParam.CurrentTheta);
	odoParam.CurrentYPos += odoParam.DeltaCurrentXPos*sin(odoParam.CurrentTheta)+odoParam.DeltaCurrentYPos*cos(odoParam.CurrentTheta);
	odoParam.CurrentTheta += odoParam.DeltaCurrentTheta;
}
