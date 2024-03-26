/*
 * AngleOptimizer.h
 *
 *  Created on: Oct 12, 2023
 *      Author: Admin
 */

#ifndef INC_ANGLEOPTIMIZER_H_
#define INC_ANGLEOPTIMIZER_H_
#include "main.h"
#include "math.h"
#include "SwerveModule.h"

float modulo360(float angle);
float modulo180(float angle);
void angopt_Cal(ModuleID ID,float input);
void angopt_QuadRantCheckOutput(ModuleID ID,float Input);
int angopt_QuadrantCheckInput(float x, float y);

#endif /* INC_ANGLEOPTIMIZER_H_ */
