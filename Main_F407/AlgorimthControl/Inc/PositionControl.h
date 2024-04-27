/*
 * PositionControl.h
 *
 *  Created on: Feb 17, 2024
 *      Author: Admin
 */

#ifndef INC_POSITIONCONTROL_H_
#define INC_POSITIONCONTROL_H_

#include "PID.h"



typedef struct trajec_Param{
	float t;
	float a0,a1,a2,a3;
	float xTrajec,xdottraject;

	float P0,Pf,tf,v0,vf;
}trajec_Param;

typedef struct pd_Param{
	float e;
	float pre;

	float kP;
	float kD;

	float uP;
	float uD;
	float uDf;
	float uDfpre;
	float Alpha;

	float u;
	float uAbove;
	float uBelow;
	float DeltaT;
}pd_Param;

void PD_SetParameter(pd_Param *pd, float kp,float kd,float alpha);
void PD_SetSaturate(pd_Param *pd, float uAbove, float uBelow);
void PD_Cal(pd_Param *pd, float Target,float Current);

void trajecPlan_SetParam(trajec_Param *trajec,float P0,float Pf,float tf,float v0,float vf);
void trajecPlan_Cal(trajec_Param *trajec);
void trajecPlan_Reset(trajec_Param *trajec);
#endif /* INC_POSITIONCONTROL_H_ */
