/*
 * ControllerPD.h
 *
 *  Created on: Jan 26, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_CONTROLLERPD_H_
#define INC_CONTROLLERPD_H_

typedef enum PD_CONTROL_TYPE {
	PD_X,
	PD_Y,
	PD_THETA,
} PD_CONTROL_TYPE;

typedef struct PDParam {
	float e;
	float pre;

	float kP;
	float kD;

	float uP;
	float uD;
	float uDf;
	float uDfpre;
	float alpha;

	float u;
	float uAbove;
	float uBelow;
} PDParam;

//void PD_SetType(PDParam pd, PD_CONTROL_TYPE pdType);
//PDParam PD_GetType(PD_CONTROL_TYPE pdType);
void PD_ResetOutput(PD_CONTROL_TYPE pdType);
float PD_GetOutput(PD_CONTROL_TYPE pdType);
float PD_GetError(PD_CONTROL_TYPE pdType);
void PD_Cal(PD_CONTROL_TYPE pdType, float Target, float Current, float SampleTime);
void PD_setParam(PD_CONTROL_TYPE pdType, float kP, float kD, float alpha);
void PD_setLimit(PD_CONTROL_TYPE pdType, float uAbove, float uBelow);

#endif /* INC_CONTROLLERPD_H_ */
