/*
 * Encoder.h
 *
 *  Created on: Jan 17, 2024
 *      Author: NamDHay
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include "stdbool.h"
typedef enum EncoderTypeMode{
	 INTERRUPT,
	 TIMER,
}EncoderTypeMode;
typedef enum EncoderCountMode{
	 MODE_X1,
	 MODE_X4,
	 MODE_ANGLE,
}EncoderCountMode;

typedef struct Encoder_t{
	EncoderTypeMode mode;
//------------------------Pins & Ports---------------//
	GPIO_TypeDef *portA;
	uint16_t pinA;
	GPIO_TypeDef *portB;
	uint16_t pinB;
//------------------------Timer & Count--------------//
	TIM_HandleTypeDef *htim;
	int32_t count_X4;
	int32_t count_X1;
	int32_t count_Pre;
	uint32_t count_PerRevol;
//------------------------Speed Val-----------------//
	float vel_Real;
	float vel_Pre;
	float vel_Fil;
//------------------------Pos Cal-------------------//
	float Degree;
	float deltaT;
}Encoder_t;

void encoder_Init(Encoder_t *enc,EncoderTypeMode mode,
					TIM_HandleTypeDef *htim, uint16_t pulPerRev, float deltaT,
					GPIO_TypeDef *portA, uint16_t pinA,
					GPIO_TypeDef *portB, uint16_t pinB);
float encoder_GetSpeed(Encoder_t *enc);
float encoder_GetPulse(Encoder_t *enc, EncoderCountMode count_Mode);
void encoder_ResetCount(Encoder_t *enc);
float encoder_GetFilterSpeedVal(Encoder_t *enc);

#endif /* INC_ENCODER_H_ */
