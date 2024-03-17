/*
 * TrajectoryPlanning.c
 *
 *  Created on: Mar 17, 2024
 *      Author: Nguyen Hai Hoang
 *      Refactor by: SpiritBoi
 */

#include "TrajectoryPlanning.h"

#define a0 pParams->a0
#define a1 pParams->a1
#define a2 pParams->a2
#define a3 pParams->a3
#define p0 pParams->p0
#define v0 pParams->v0
#define t  pParams->t
#define tp pParams->tp
#define pf tp.pf
#define tf tp.tf
#define vf tp.vf
#define xTrajec  		pParams->xTrajec
#define xDotTraject pParams->xDotTraject

void trajectplan_Calculate(TrajectoryCalculateParameters *pParams) {
	a0 = p0;
	a1 = v0;
	a2 = (3 / (tf * tf)) * (pf - p0) - (2 / tf) * v0 - (1 / tf) * vf;
	a3 = (-2 / (tf * tf * tf)) * (pf - p0) + (1 / (tf * tf)) * (vf + v0);

	if(t > tf) t = tf;

	xTrajec = a0 + (a1 * t) + (a2 * t * t) + (a3 * t * t * t);
	xDotTraject = a1 + (2 * a2 * t) + (3 * a3 * t * t);

	if(pf == p0) {
		xTrajec = pf;
		xDotTraject = 0;
	}
}

void trajectplan_SetCalculateParameters(TrajectoryCalculateParameters *pParams, float _p0, float _v0, TrajectPlanningPoint _tp)
{
	t = 0;
	p0 = _p0;
	pf = _pf;
	tp = _tp;
}
