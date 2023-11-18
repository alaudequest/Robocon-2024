/*
 * Encoder.c
 *
 *  Created on: Sep 4, 2023
 *      Author: defaultuser0
 */
#include "main.h"
#include "Encoder.h"

void EncoderSetting(EncoderRead *enc,TIM_HandleTypeDef *htim,int count_PerRevol,double deltaT)
{
	enc->htim = htim;
	enc->count_PerRevol = count_PerRevol;
	enc->deltaT = deltaT;
}
void SpeedReadOnly(EncoderRead *enc)
{
	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = (enc->count_X4/enc->deltaT)/(enc->count_PerRevol*4)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_X4 = 0;
}



void SpeedReadNonReset(EncoderRead *enc){

	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = ((enc->count_X4-enc->count_Pre)/enc->deltaT)/(enc->count_PerRevol*4)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_Pre = enc->count_X4;
}

double CountRead(EncoderRead *enc,uint8_t count_mode){
	enc->count_Mode = count_mode;
	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);

	if (enc->count_Mode == count_ModeX4)
	{
		return enc->count_X4;
	}else if (enc->count_Mode == count_ModeX1)
	{
		enc->count_X1 = enc->count_X4/4;
		return enc->count_X1;
	}else if (enc->count_Mode == count_ModeDegree)
	{
		enc->Degree = enc->count_X4*360/(enc->count_PerRevol*4);
		return enc->Degree;
	}else {
		return 0;
	}
}

void ResetCount(EncoderRead *enc,uint8_t command)
{
	if (command == 1)
	{
		__HAL_TIM_SET_COUNTER(enc->htim,0);
		enc->count_X4 = 0;
		enc->count_X1 = 0;
	}
}


