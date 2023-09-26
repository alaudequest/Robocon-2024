/*
 * Encoder.c
 *
 *  Created on: Sep 16, 2023
 *      Author: KHOA
 */

#include "Encoder.h"
#include "stdbool.h"


void encoder_Init(Encoder_t *enc,TIM_HandleTypeDef *htim, uint16_t pulPerRev, uint16_t deltaT)
{
	enc->htim = htim;
	enc->count_PerRevol = pulPerRev;
	enc->deltaT = deltaT;
}



double encoder_GetSpeed(Encoder_t *enc, bool resetPulse)
{
	enc->count_X4 += (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = (enc->count_X4/enc->deltaT)/(enc->count_PerRevol*4)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;

	enc->count_Pre = enc->count_X4;
	enc->count_X4 = 0;
	return enc->vel_Real;
}

double encoder_GetPulse(Encoder_t *enc, bool resetPulse)
{
	enc->count_Mode = count_ModeX1;
	enc->count_X4 += (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);
	if(resetPulse) __HAL_TIM_SET_COUNTER(enc->htim,0);

	if (enc->count_Mode == count_ModeX4) return enc->count_X4;
	else if (enc->count_Mode == count_ModeX1) return enc->count_X4/4;
	else if (enc->count_Mode == count_ModeDegree) return enc->count_X4*360/(enc->count_PerRevol*4);
	return 0;
}
void encoder_ResetCount(Encoder_t *enc)
{
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->count_X4 = 0;
}


