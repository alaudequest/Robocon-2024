/*
 * PID.h
 *
 *  Created on: Sep 6, 2023
 *      Author: defaultuser0
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

//------------------------Begin: Struct of OutSumValue-----------------------------//
typedef struct PID_Param{
//---------Input Parameters-----------//
	float Target;
	float CurrVal;
	float e;
	float e_Pre;
	float deltaT;
//---------Propotion Parameters-------//
	float kP;
	float uP;
//---------Intergral Parameters-------//
	float kI;
	float uI;
	float uI_Pre;
	int uI_AboveLimit;
	int uI_BelowLimit;
//---------Derivative Parameters------//
	float kD;
	float uD;
	float uD_Fil;
	float uD_FilPre;
	float alpha;
//---------Sum Parameters-------------//
	float u;
	float u_AboveLimit;
	float u_BelowLimit;
}PID_Param;
//------------------------End: Struct of OutSumValue-------------------------------//

//------------------------Begin: Function of Pid-----------------------------------//
void Pid_SetParam(PID_Param *pid,float kP,float kI,float kD,float alpha,float deltaT,float uI_AboveLimit,float uI_BelowLimit,float u_AboveLimit,float u_UnderLimit);
void Pid_uI_PreSetParam(PID_Param *pid,float uI_AboveLimit,float uI_BelowLimit);
void Pid_u_PresetParam(PID_Param *pid,float u_AboveLimit,float u_BelowLimit);
void Pid_Term_PresetParam(PID_Param *pid,float kP,float kI,float kD);
void Pid_Cal(PID_Param *pid,float Target,float CurrVal);
//------------------------End: Function of Pid-------------------------------------//

#endif /* INC_PID_H_ */
