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

typedef enum EncoderCountMode{
	 MODE_X1,
	 MODE_X4,
	 MODE_ANGLE,
}EncoderCountMode;

typedef struct Encoder_t{
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

void encoder_Init_InterruptMode(Encoder_t *enc, uint16_t pulPerRev, float deltaT);
void encoder_Init_EncoderMode(Encoder_t *enc,TIM_HandleTypeDef *htim, uint16_t pulPerRev, float deltaT);
float encoder_GetSpeed(Encoder_t *enc);
void encoder_ResetCount(Encoder_t *enc);
float encoder_GetFilterSpeedVal(Encoder_t *enc);

//float encoder_GetAngle(Encoder_t *enc);

#endif /* INC_ENCODER_H_ */
