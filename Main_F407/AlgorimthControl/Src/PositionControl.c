/*
 * PositionControl.c
 *
 *  Created on: Feb 17, 2024
 *      Author: Admin
 */
#include "PositionControl.h"

void PD_SetParameter(pd_Param *pd, float kp,float kd,float alpha)
{
	pd->kP = kp;
	pd->kD = kd;
	pd->Alpha = alpha;
}

void PD_SetSaturate(pd_Param *pd, float uAbove, float uBelow)
{
	pd->uAbove = uAbove;
	pd->uBelow = uBelow;
}

void PD_Cal(pd_Param *pd, float Target,float Current)
{
	pd->e = Target - Current;
	pd->uP = pd->kP*pd->e;
	pd->uD = pd->kD*(pd->e - pd->pre)/pd->DeltaT;
	pd->uDf = (1-pd->Alpha)*pd->uDfpre+(pd->Alpha)*pd->uD;
	pd->uDfpre = pd->uDf;
	pd->pre = pd->e;

	pd->u = pd->uP + pd->uD;
	if(pd->u > pd->uAbove)pd->u = pd->uAbove;
	else if (pd->u < pd->uBelow)pd->u = pd->uBelow;
}

void trajecPlan_Cal(trajec_Param *trajec){
	trajec->a0 = trajec->P0;
	trajec->a1 = trajec->v0;
	trajec->a2 = (3/(trajec->tf*trajec->tf))*(trajec->Pf - trajec->P0) - (2/trajec->tf)*trajec->v0 - (1/trajec->tf)*trajec->vf;
	trajec->a3 = (-2/(trajec->tf*trajec->tf*trajec->tf))*(trajec->Pf - trajec->P0) + (1/(trajec->tf*trajec->tf))*(trajec->vf + trajec->v0);

	if (trajec->t > trajec->tf) trajec->t = trajec->tf;

	trajec->xTrajec = trajec->a0 + trajec->a1*trajec->t + trajec->a2*trajec->t*trajec->t + trajec->a3*trajec->t*trajec->t*trajec->t;
	trajec->xdottraject = trajec->a1 + 2*trajec->a2*trajec->t + 3* trajec->a3*trajec->t*trajec->t;

	if(trajec->Pf == trajec->P0){
		trajec->xTrajec = trajec->Pf;
		trajec->xdottraject = 0;
	}
}

void trajecPlan_SetParam(trajec_Param *trajec,float P0,float Pf,float tf,float v0,float vf)
{
	trajec -> t = 0;
	trajec -> P0 = P0;
	trajec -> Pf = Pf;
	trajec -> tf = tf;
	trajec -> v0 = v0;
	trajec -> vf = vf;
}

void trajecPlan_Reset(trajec_Param *trajec)
{
	trajec -> t = 0;
	trajec -> P0 = 0;
	trajec -> Pf = 0;
	trajec -> tf = 1;
	trajec -> v0 = 0;
	trajec -> vf = 0;
	trajec -> xTrajec = 0;
	trajec -> xdottraject = 0;
	trajec->a0 = 0;
	trajec->a1 = 0;
	trajec->a2 = 0;
	trajec->a3 = 0;
}
