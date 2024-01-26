/*
 * ControllerPD.c
 *
 *  Created on: Jan 26, 2024
 *      Author: SpiritBoi
 */

#include "ControllerPD.h"

PDParam pDX;
PDParam pDY;
PDParam pDTheta;
bool isEnable = false;
void PD_On() {
	isEnable = true;
}

void PD_Off() {
	isEnable = false;
}

void PD_SetType(PDParam pd, PD_CONTROL_TYPE pdType) {
	switch (pdType) {
		case PD_X:
			pDX = pd;
		break;
		case PD_Y:
			pDY = pd;
		break;
		case PD_THETA:
			pDTheta = pd;
		break;
		default:
			break;
	}
}

PDParam PD_GetType(PD_CONTROL_TYPE pdType) {
	switch (pdType) {
		case PD_X:
			return pDX;
		break;
		case PD_Y:
			return pDY;
		break;
		case PD_THETA:
			return pDTheta;
		break;
		default:
			break;
	}
	PDParam pd = {0};
	return pd;
}

void PD_ResetOutput(PD_CONTROL_TYPE pdType) {
	PDParam pd = PD_GetType(pdType);
	pd.u = 0;
	PD_SetType(pd, pdType);
}

float PD_GetOutput(PD_CONTROL_TYPE pdType) {
	return PD_GetType(pdType).u;
}

float PD_GetError(PD_CONTROL_TYPE pdType) {
	return PD_GetType(pdType).e;
}

void PD_Cal(PD_CONTROL_TYPE pdType, float Target, float Current, float SampleTime)
{
	if (!isEnable) return;
	PDParam pd = PD_GetType(pdType);
	pd.e = Target - Current;
	pd.uP = pd.kP * pd.e;
	pd.uD = pd.kD * (pd.e - pd.pre) / SampleTime;
	pd.uDf = (1 - pd.alpha) * pd.uDfpre + (pd.alpha) * pd.uD;
	pd.uDfpre = pd.uDf;
	pd.pre = pd.e;

	pd.u = pd.uP + pd.uD;
	if (pd.u > pd.uAbove)
		pd.u = pd.uAbove;
	else if (pd.u < pd.uBelow) pd.u = pd.uBelow;
	PD_SetType(pd, pdType);
}

void PD_setParam(PD_CONTROL_TYPE pdType, float kP, float kD, float alpha)
{
	PDParam pd = PD_GetType(pdType);
	pd.kP = kP;
	pd.kD = kD;
	pd.alpha = alpha;
	PD_SetType(pd, pdType);
}

void PD_setLimit(PD_CONTROL_TYPE pdType, float uAbove, float uBelow)
{
	PDParam pd = PD_GetType(pdType);
	pd.uAbove = uAbove;
	pd.uBelow = uBelow;
	PD_SetType(pd, pdType);
}

