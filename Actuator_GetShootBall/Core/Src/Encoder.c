/*
 * Encoder.c
 *
 *  Created on: Jan 17, 2024
 *      Author: NamDHay
 */

#include "Encoder.h"

void encoder_Init(Encoder_t *enc,EncoderTypeMode mode,
					TIM_HandleTypeDef *htim, uint16_t pulPerRev, float deltaT,
					GPIO_TypeDef *portA, uint16_t pinA,
					GPIO_TypeDef *portB, uint16_t pinB)
{
	enc->mode = mode;
//------------------------Timer Mode-----------------//
	enc->htim = htim;
	enc->count_PerRevol = pulPerRev;
	enc->deltaT = deltaT;
//------------------------Interrupt Mode-------------//
	enc->portA = portA;
	enc->portB = portB;
	enc->pinA  = pinA;
	enc->pinB  = pinB;
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
	return 0;
}

void encoder_ResetCount(Encoder_t *enc)
{
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->count_X4 = 0;
	enc->count_X1 = 0;
	enc->vel_Pre  = 0;
	enc->vel_Real = 0;
}

float encoder_GetFilterSpeedVal(Encoder_t *enc){return enc->vel_Fil;}
