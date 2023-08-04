/*
 * PID.h
 *
 *  Created on: Jul 30, 2023
 *      Author: LENOVO
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

#define PID_EN
#define ENC_EN
#define MOTOR_EN


#ifdef PID_EN

//------------------------Begin: Struct of OutSumValue-----------------------------//
typedef struct PID_Param{
//---------Input Parameters-----------//
	double Target;
	double CurrVal;
	double e;
	double e_Pre;
	double deltaT;
//---------Propotion Parameters-------//
	double kP;
	double uP;
//---------Intergral Parameters-------//
	double kI;
	double uI;
	double uI_Pre;
	int uI_AboveLimit;
	int uI_BelowLimit;
//---------Derivative Parameters------//
	double kD;
	double uD;
	double uD_Fil;
	double uD_FilPre;
	double alpha;
//---------Sum Parameters-------------//
	double u;
	double u_AboveLimit;
	double u_BelowLimit;
}PID_Param;
//------------------------End: Struct of OutSumValue-------------------------------//

//------------------------Begin: Function of Pid-----------------------------------//
void Pid_SetParam(PID_Param *pid,double kP,double kI,double kD,double alpha,double deltaT,double uI_AboveLimit,double uI_BelowLimit,double u_AboveLimit,double u_UnderLimit);
void Pid_uI_PreSetParam(PID_Param *pid,double uI_AboveLimit,double uI_BelowLimit);
void Pid_u_PresetParam(PID_Param *pid,double u_AboveLimit,double u_BelowLimit);
void Pid_Term_PresetParam(PID_Param *pid,double kP,double kI,double kD);
void Pid_Cal(PID_Param *pid,double Target,double CurrVal);
//------------------------End: Function of Pid-------------------------------------//
#endif



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

#endif /* INC_PID_H_ */
