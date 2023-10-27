/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Hoang May
 */
#include "InverseKinematic.h"
#include "AngleOptimizer.h"

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
	float rawAngle = atan2(vect.wheelVelY,vect.wheelVelX)*180/M_PI;
	angopt_Cal(ID, rawAngle);
	//------------------------------------------------------------------

}

float invkine_CalSpeedVectorControl(ModuleID ID)
{
	float temp;
	WheelVector vect = swer_GetWheelVector(ID);
	Angle_Opt_Param angopt = swer_GetOptAngle(ID);
	angopt_QuadRantCheckOutput(ID,angopt.currentAngle*M_PI/180);
	temp = (float)angopt.direct * (1/(float)ROBOT_WHEEL_RADIUS_METER) * (sqrt(pow(vect.wheelVelX,2) + pow(vect.wheelVelY,2))) / 0.1047198;
	return temp;
}

HAL_StatusTypeDef  invkine_Implementation(ModuleID ID, float u, float v, float r,void (*ptnCpltCallback)(ModuleID,float, float))
{
	static float velocity = 0;
	invkine_CalWheelVector(ID, u, v, r);
	invkine_CalOptAngle(ID);
	velocity = invkine_CalSpeedVectorControl(ID);
	Angle_Opt_Param angopt = swer_GetOptAngle(ID);
	ptnCpltCallback(ID,velocity,angopt.currentAngle);
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
