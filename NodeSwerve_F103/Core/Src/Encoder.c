/*
 * Encoder.c
 *
 *  Created on: Sep 16, 2023
 *      Author: KHOA
 */

#include "Encoder.h"
#include "stdbool.h"


void encoder_Init(Encoder_t *enc,TIM_HandleTypeDef *htim, uint16_t pulPerRev, float deltaT)
{
	enc->htim = htim;
	enc->count_PerRevol = pulPerRev;
	enc->deltaT = deltaT;
}



float encoder_GetSpeed(Encoder_t *enc)
{
	enc->count_X4 += (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = ((enc->count_X4 - enc->count_Pre)/enc->deltaT)/(enc->count_PerRevol*4)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_Pre = enc->count_X4;
	return enc->vel_Real;
}

float encoder_GetPulse(Encoder_t *enc, EncoderCountMode count_Mode)
{
	enc->count_X4 += (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	if (count_Mode == MODE_X4) return enc->count_X4;
	else if (count_Mode == MODE_X1) return (float)enc->count_X4/4;
	else if (count_Mode == MODE_ANGLE) {
		enc->Degree = (float)((float)enc->count_X4*360.0/((float)enc->count_PerRevol*4.0));
		return enc->Degree;
	}
	return 0;
}
void encoder_ResetCount(Encoder_t *enc)
{
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->count_X4 = 0;
	enc->vel_Pre = 0;
	enc->vel_Real = 0;
	enc->count_Pre = 0;
}

float encoder_GetFilterSpeedVal(Encoder_t *enc){return enc->vel_Fil;}

