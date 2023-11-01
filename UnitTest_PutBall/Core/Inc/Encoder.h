/*
 * Encoder.h
 *
 *  Created on: Sep 4, 2023
 *      Author: defaultuser0
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

//------------------------Begin: Struct of Encoder Read----------------------------//

typedef struct EncoderRead{
//------------------------Timer & Count--------------//
	TIM_HandleTypeDef *htim;
	int16_t count_Timer ;
	int count_X4;
	int count_X1;
	int count_Pre;
	int count_PerRevol;
	uint8_t count_Mode;
//------------------------Speed Val-----------------//
	float vel_Real;
	float vel_Pre;
	float vel_Fil;
//------------------------Pos Cal-------------------//
	float Degree;
	float deltaT;
}EncoderRead;

//------------------------End: Struct of Encoder Read------------------------------//

//------------------------Begin: Function of Encoders------------------------------//
#define count_ModeX1 0
#define count_ModeX4 1
#define count_ModeDegree 2

void EncoderSetting(EncoderRead *enc,TIM_HandleTypeDef *htim,int count_PerRevol,float deltaT);
void SpeedReadOnly(EncoderRead *enc);
void SpeedReadNonReset(EncoderRead *enc);
float CountRead(EncoderRead *enc,uint8_t count_mode);
void ResetCount(EncoderRead *enc,uint8_t command);
//------------------------End: Function of Encoders---------------------------------//

#endif /* INC_ENCODER_H_ */
