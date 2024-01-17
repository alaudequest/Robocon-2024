/*
 * Encoder.c
 *
 *  Created on: Jan 17, 2024
 *      Author: NamDHay
 */

#include "Encoder.h"

void encoder_Init(Encoder_t *enc,
					TIM_HandleTypeDef *htim, uint16_t pulPerRev, float deltaT,
					GPIO_TypeDef portA, uint16_t pinA,
					GPIO_TypeDef portB, uint16_t pinB)
{
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

}

float encoder_GetPulse(Encoder_t *enc, EncoderCountMode count_Mode)
{

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
