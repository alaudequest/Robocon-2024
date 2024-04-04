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

	float rawAngle = atan2(vect.wheelVelY,vect.wheelVelX)*180.0/M_PI;
	velocity = invkine_CalSpeedVectorControl(ID);

	if(u == 0&&v==0&&r==0)rawAngle= vect.PreAngle;
	ptnCpltCallback(ID,velocity,rawAngle);

	vect.PreAngle = rawAngle;
	swer_SetWheelVector(ID,vect);
	return HAL_OK;
}

void invkine_Begin(){InvCalcStep = INV_PROC_BEGIN;}
InverseKinematicProcedure invkine_GetStep(){return InvCalcStep;}



InverseKinematicProcedure invkine_GetInvCalStep(){return InvCalcStep;}
void invkine_SetInvCalStep(InverseKinematicProcedure Step){InvCalcStep = Step;}
