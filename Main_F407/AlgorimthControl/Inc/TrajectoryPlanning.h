/*
 * TrajectoryPlanning.h
 *
 *  Created on: Mar 17, 2024
 *      Author: Nguyen Hai Hoang
 *      Refactor by: SpiritBoi
 */

#ifndef INC_TRAJECTORYPLANNING_H_
#define INC_TRAJECTORYPLANNING_H_

#include "main.h"

typedef struct TrajectPlanningPoint {
	float pf;
	float tf;
	float vf;
	float ReachOffset;
} TrajectPlanningPoint;

typedef struct TrajectoryCalculateParameters {
	float t;
	float a0, a1, a2, a3;
	float xTrajec, xDotTraject;

	float p0, v0;
	TrajectPlanningPoint tp;
} TrajectoryCalculateParameters;

typedef uint8_t TrajectoryStage;

void trajectplan_Calculate(TrajectoryCalculateParameters *pParams);
void trajectplan_SetCalculateParameters(TrajectoryCalculateParameters *pParams, float _p0, float _v0, TrajectPlanningPoint _tp);

#endif /* INC_TRAJECTORYPLANNING_H_ */
