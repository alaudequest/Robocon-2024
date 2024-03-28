/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Hoang May
 */
#include "InverseKinematic.h"
#include "AngleOptimizer.h"
#include "main.h"
int Count;
extern TIM_HandleTypeDef htim10;
InverseKinematicProcedure InvCalcStep = 0;

void invkine_CalWheelVector(ModuleID ID, float u, float v, float r){
	pVectorCalXY pVectXY = swer_GetFuncHandle(ID);
	pVectXY(u,v,r);
}

void invkine_CalOptAngle(ModuleID ID){
	WheelVector vect = swer_GetWheelVector(ID);
	Angle_Opt_Param angopt = swer_GetOptAngle(ID);
	angopt.Case1 = angopt_QuadrantCheckInput(vect.wheelVelX,vect.wheelVelY);
	swer_SetOptAngle(ID, angopt);

	//------------------------------------------------------------------
	float rawAngle = atan2(vect.wheelVelY,vect.wheelVelX)*180.0/M_PI;
	angopt_Cal(ID, rawAngle);
	//------------------------------------------------------------------

}

float invkine_CalSpeedVectorControl(ModuleID ID)
{
	float temp;
	WheelVector vect = swer_GetWheelVector(ID);
	Angle_Opt_Param angopt = swer_GetOptAngle(ID);
	angopt_QuadRantCheckOutput(ID,angopt.currentAngle*M_PI/180);
	temp = (float)angopt.direct  * (sqrt(pow(vect.wheelVelX,2) + pow(vect.wheelVelY,2))) ;
	return temp;
}

HAL_StatusTypeDef  invkine_Implementation(ModuleID ID, float u, float v, float r,void (*ptnCpltCallback)(ModuleID,float, float))
{
//	HAL_TIM_Base_Start(&htim10);
//	__HAL_TIM_SET_COUNTER(&htim10,0);
	static float velocity = 0;

	invkine_CalWheelVector(ID, u, v, r);
	if(u == 0&&v==0&&r==0)__NOP();
	else invkine_CalOptAngle(ID);

	velocity = invkine_CalSpeedVectorControl(ID);
	Angle_Opt_Param angopt = swer_GetOptAngle(ID);
	if(u == 0&&v==0&&r==0)angopt.currentAngle= angopt.PreCurrAngle;
//	Count = __HAL_TIM_GET_COUNTER(&htim10);
//	HAL_TIM_Base_Stop(&htim10);
	ptnCpltCallback(ID,velocity,angopt.currentAngle);
	angopt.PreCurrAngle = angopt.currentAngle;
	swer_SetOptAngle(ID, angopt);

	return HAL_OK;
}

void invkine_Begin(){InvCalcStep = INV_PROC_BEGIN;}
InverseKinematicProcedure invkine_GetStep(){return InvCalcStep;}

void invkine_Test(){
//	invkine_Implementation(MODULE_ID_1,1,0,0);
//	invkine_Implementation(MODULE_ID_2,1,0,0);
//	invkine_Implementation(MODULE_ID_3,1,0,0);
//	invkine_Implementation(MODULE_ID_4,1,0,0);
	InvCalcStep = 1;
}

InverseKinematicProcedure invkine_GetInvCalStep(){return InvCalcStep;}
void invkine_SetInvCalStep(InverseKinematicProcedure Step){InvCalcStep = Step;}
