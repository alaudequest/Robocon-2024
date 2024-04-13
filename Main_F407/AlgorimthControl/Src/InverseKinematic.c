/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Hoang May
 */
#include "InverseKinematic.h"

#include "main.h"
int Count;
extern TIM_HandleTypeDef htim10;
float rawAngle;
InverseKinematicProcedure InvCalcStep = 0;

void invkine_CalWheelVector(ModuleID ID, float u, float v, float r){
	pVectorCalXY pVectXY = swer_GetFuncHandle(ID);
	pVectXY(u,v,r);
}

float invkine_CalSpeedVectorControl(ModuleID ID)
{
	float temp;
	WheelVector vect = swer_GetWheelVector(ID);
	temp = (60.0/(ROBOT_WHEEL_RADIUS_METER*2.0*M_PI)) * (sqrt(pow(vect.wheelVelX,2) + pow(vect.wheelVelY,2))) ;
	return temp;
}

HAL_StatusTypeDef  invkine_Implementation(ModuleID ID, float u, float v, float r,void (*ptnCpltCallback)(ModuleID,float, float))
{
	static float velocity = 0;

	invkine_CalWheelVector(ID, u, v, r);
	WheelVector vect = swer_GetWheelVector(ID);


	velocity = invkine_CalSpeedVectorControl(ID);
	Angle_Opt_Param angopt = swer_GetOptAngle(ID);

	if(u == 0&&v==0&&r==0)angopt.currentAngle= angopt.PreCurrAngle;
	else{
		angopt.currentAngle =  atan2(vect.wheelVelY,vect.wheelVelX)*180.0/M_PI;
	}
	ptnCpltCallback(ID,velocity,angopt.currentAngle);
	angopt.PreCurrAngle = angopt.currentAngle;
	swer_SetOptAngle(ID, angopt);

	return HAL_OK;
}

void invkine_Begin(){InvCalcStep = INV_PROC_BEGIN;}
InverseKinematicProcedure invkine_GetStep(){return InvCalcStep;}



InverseKinematicProcedure invkine_GetInvCalStep(){return InvCalcStep;}
void invkine_SetInvCalStep(InverseKinematicProcedure Step){InvCalcStep = Step;}
