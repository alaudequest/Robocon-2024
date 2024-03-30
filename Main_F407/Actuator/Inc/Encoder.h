/*
 * Encoder.h
 *
 *  Created on: Sep 16, 2023
 *      Author: KHOA
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

//------------------------Begin: Struct of Encoder Read----------------------------//
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
void encoder_ResetCount(Encoder_t *enc);
void encoder_Init(Encoder_t *enc,TIM_HandleTypeDef *htim, uint16_t pulPerRev, float deltaT);
float encoder_GetSpeed(Encoder_t *enc);
float encoder_GetPulse(Encoder_t *enc, EncoderCountMode count_Mode);
float encoder_GetFilterSpeedVal(Encoder_t *enc);
void VelCal(Encoder_t *enc,int count_X1,uint32_t count_PerRevol, float deltaT );
#endif /* INC_ENCODER_H_ */
