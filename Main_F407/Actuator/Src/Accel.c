/*
 * Accel.c
 *
 *  Created on: Nov 6, 2023
 *      Author: kaychip
 */

#include "Accel.h"
#include "stdlib.h"


#define accelValue accel->accelValue
#define currentVelocity accel->currentVelocity
#define preVelocityVal accel->preVelocityVal
#define preTargeVelocity accel->preTargeVelocity

AccelerationGradient grad;

void accel_Calculate(AccelerationController_t *accel, float targetVelocity, float targetTime, float deltaT)
{
	if(targetVelocity != preTargeVelocity)
	{
		if(targetVelocity < preTargeVelocity)
			grad = ACCEL_ON_RAMP_DOWN;
		else if(targetVelocity > preTargeVelocity)
			grad = ACCEL_ON_RAMP_UP;
		else
			grad = ACCEL_NO_CHANGE;
		accelValue = (targetVelocity - currentVelocity)/targetTime;
		preTargeVelocity = targetVelocity;
	}
	currentVelocity += accelValue * deltaT; //accumulate velocity over time

	// if current velocity fall below or higher up the target velocity, pin the current value to target value
	if((currentVelocity < targetVelocity && grad == ACCEL_ON_RAMP_DOWN) ||
			(currentVelocity > targetVelocity && grad == ACCEL_ON_RAMP_UP))
	currentVelocity = targetVelocity;

}
