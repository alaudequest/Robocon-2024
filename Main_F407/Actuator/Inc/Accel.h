/*
 * Accel.h
 *
 *  Created on: Nov 6, 2023
 *      Author: khoac
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "main.h"

typedef struct Accel_Param
{
	float accel;
	float vel_controller;
	float vel_Pre;
	float target_vel_Pre;
	uint8_t Flag;
} AccelParam_t;

void Accel_Cal(AccelParam_t *accel, float target_vel, float target_time);

#endif /* INC_ACCEL_H_ */
