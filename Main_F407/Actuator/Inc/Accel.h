/*
 * Accel.h
 *
 *  Created on: Nov 6, 2023
 *      Author: khoac
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "main.h"

typedef struct AccelerationController_t
{
	float accelValue;
	float currentVelocity;
	float preVelocityVal;
	float preTargeVelocity;
} AccelerationController_t;

typedef enum AccelerationGradient {
	ACCEL_ON_RAMP_UP,
	ACCEL_NO_CHANGE,
	ACCEL_ON_RAMP_DOWN,
} AccelerationGradient;

void accel_Calculate(AccelerationController_t *accel, float targetVelocity, float targetTime, float deltaT);

#endif /* INC_ACCEL_H_ */
