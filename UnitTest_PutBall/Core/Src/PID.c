/*
 * PID.c
 *
 *  Created on: Sep 6, 2023
 *      Author: defaultuser0
 */
#include <PID.h>
#include "stdlib.h"

//-----------------------------------------------Begin: Setting Parameter for PID------------------------------------------//
void Pid_SetParam(PID_Param *pid,float kP,float kI,float kD,float alpha,float deltaT,float uI_AboveLimit,float uI_BelowLimit,float u_AboveLimit,float u_BelowLimit)
{
//----------------------Term-----------------------//
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->alpha = alpha;
//----------------------Sample Time----------------//
	pid->deltaT = deltaT;
//----------------------Limit----------------------//
	pid->uI_AboveLimit = uI_AboveLimit;
	pid->uI_BelowLimit = uI_BelowLimit;
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}

//-----------------------------------------------End: Setting Parameter for PID--------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------Begin: Change Parameter for PID-------------------------------------------//

void Pid_uI_PreSetParam(PID_Param *pid,float uI_AboveLimit,float uI_BelowLimit)
{
	pid->uI_AboveLimit = uI_AboveLimit;
	pid->uI_BelowLimit = uI_BelowLimit;
}



void Pid_u_PresetParam(PID_Param *pid,float u_AboveLimit,float u_BelowLimit)
{
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}



void Pid_Term_PresetParam(PID_Param *pid,float kP,float kI,float kD)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}

//-----------------------------------------------End: Change Parameter for PID---------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------Begin: Calculating PID---------------------------------------------------//

void Pid_Cal(PID_Param *pid,float Target,float CurrVal)
{
//-----------------------Input-------------------------//
	pid->Target = Target;
	pid->CurrVal = CurrVal;
	pid->e = pid->Target - pid->CurrVal;

//-----------------------Propotion Term----------------//
	pid->uP = pid->kP*pid->e;

//-----------------------Integral Term-----------------//
	pid->uI = pid->uI_Pre + pid->kI*pid->e*pid->deltaT;
	pid->uI = pid->uI > pid->uI_AboveLimit ? pid->uI_AboveLimit : pid->uI;
	pid->uI = pid->uI < pid->uI_BelowLimit ? pid->uI_BelowLimit : pid->uI;

//-----------------------Derivative Term---------------//
	pid->uD = pid->kD*(pid->e - pid->e_Pre)/pid->deltaT;
	pid->uD_Fil = (1-pid->alpha)*pid->uD_FilPre+pid->alpha*pid->uD;

//-----------------------Previous Value----------------//
	pid->e_Pre = pid->e;
	pid->uI_Pre = pid->uI;
	pid->uD_FilPre = pid->uD_Fil;

//-----------------------Sum---------------------------//
	pid->u = pid->uP + pid->uI + pid->uD;
	pid->u = pid->u > pid->u_AboveLimit ? pid->u_AboveLimit : pid->u;
	pid->u = pid->u < pid->u_BelowLimit ? pid->u_BelowLimit : pid->u;

//	return pid->u;
}

//-----------------------------------------------End: Calculating PID-----------------------------------------------------//
