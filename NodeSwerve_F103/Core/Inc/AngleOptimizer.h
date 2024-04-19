/*
 * AngleOptimizer.h
 *
 *  Created on: Apr 1, 2024
 *      Author: Admin
 */

#ifndef INC_ANGLEOPTIMIZER_H_
#define INC_ANGLEOPTIMIZER_H_

#include "main.h"
#include "math.h"
#include "SwerveModule.h"

typedef struct Angle_Opt_Param{
	int direct;
	float currentAngle;
	float deltaAngle;
	float outputAngle;
	float preAngle;
	float calInput;
	float preCal;
	float deltaCal;
	float OffsetAngle;


	int Case1;
	int Case2;
	float PreCurrAngle;
}Angle_Opt_Param;

float absf(float num);
float modulo360(float angle);
float modulo180(float angle);
void angopt_Cal(float input);
void angopt_QuadRantCheckOutput(float Input);
int angopt_QuadrantCheckInput(float x, float y);
int angopt_QuadRantCheckOutput2(float Input1,float Input2);
float angopt_GetOptAngle();
#endif /* INC_ANGLEOPTIMIZER_H_ */
