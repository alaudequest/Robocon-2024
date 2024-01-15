/*
 * Accel.c
 *
 *  Created on: Nov 6, 2023
 *      Author: kaychip
 */

#include "Accel.h"
#include "stdlib.h"
#define deltaT 0.01

void Accel_Cal(Accel_Param *accel, float target_vel, float target_time)
{
	if(target_vel != accel->target_vel_Pre)
	{
		if(target_vel < accel->target_vel_Pre)
		{
			accel->Flag = 1;
		}
		else
		{
			accel->Flag = 0;
		}
		accel->accel = (target_vel - accel->vel_controller)/target_time;
		accel->target_vel_Pre = target_vel;
	}
		accel->vel_controller +=  accel->accel * deltaT;

			if(accel->Flag)
			{
				if(accel->vel_controller < target_vel) accel->vel_controller = target_vel;
			}
			else
			{
				if(accel->vel_controller > target_vel) accel->vel_controller = target_vel;
			}
}
