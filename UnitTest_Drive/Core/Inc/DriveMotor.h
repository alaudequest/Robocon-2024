/*
 * DriveMotor.h
 *
 *  Created on: Sep 5, 2023
 *      Author: defaultuser0
 */

#ifndef INC_DRIVEMOTOR_H_
#define INC_DRIVEMOTOR_H_

#include "main.h"

#define ENC_EN
#define MOTOR_EN

#ifdef ENC_EN
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
	double vel_Real;
	double vel_Pre;
	double vel_Fil;
//------------------------Pos Cal-------------------//
	double Degree;
	double deltaT;
}EncoderRead;

//------------------------End: Struct of Encoder Read------------------------------//

//------------------------Begin: Function of Encoders------------------------------//
#define count_ModeX1 0
#define count_ModeX4 1
#define count_ModeDegree 2

void EncoderSetting(EncoderRead *enc,TIM_HandleTypeDef *htim,int count_PerRevol,double deltaT);
void SpeedReadOnly(EncoderRead *enc);
void SpeedReadNonReset(EncoderRead *enc);
double CountRead(EncoderRead *enc,uint8_t count_mode);
void ResetCount(EncoderRead *enc,uint8_t command);
//------------------------End: Function of Encoders---------------------------------//
#endif



#ifdef MOTOR_EN

#define motor_Reserve 1000
#define motor_Normal 1

typedef struct MotorDrive{
	TIM_HandleTypeDef *htim1;
	TIM_HandleTypeDef *htim2;
	int Input;
	int8_t Dir;
	uint16_t Pwm;
	uint16_t Mode;
	unsigned int Channel1;
	unsigned int Channel2;
}MotorDrive;


void DC_Drive_BTS(MotorDrive *motor,TIM_HandleTypeDef *htim1,uint16_t Mode,int Input,unsigned int Channel1,unsigned int Channel2);
void BLDC_Drive_RedBoard(MotorDrive *motor,TIM_HandleTypeDef *htim1,int Input,unsigned int Channel1);

#endif


#endif /* INC_DRIVEMOTOR_H_ */
