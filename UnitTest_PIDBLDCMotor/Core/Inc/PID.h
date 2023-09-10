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
	double Target;
	double CurrVal;
	double e;
	double e_Pre;
	double deltaT;
//---------Propotion Parameters-------//
	float kP;
	double uP;
//---------Intergral Parameters-------//
	float kI;
	double uI;
	double uI_Pre;
	int uI_AboveLimit;
	int uI_BelowLimit;
//---------Derivative Parameters------//
	float kD;
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

#endif /* INC_PID_H_ */
