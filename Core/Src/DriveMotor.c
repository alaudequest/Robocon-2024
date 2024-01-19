/*
 * DriveMotor.c
 *
 *  Created on: Sep 5, 2023
 *      Author: defaultuser0
 */
#include "DriveMotor.h"
#include "stdlib.h"

void DC_Drive(MotorDrive *motor,TIM_HandleTypeDef *htim1,int Input,unsigned int Channel1)
{
	motor->htim1 = htim1;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;

		if(Input > 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,0);
		}

}
