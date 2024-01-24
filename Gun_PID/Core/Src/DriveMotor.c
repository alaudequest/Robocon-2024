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
void Gun_Drive(MotorDrive *motor,TIM_HandleTypeDef *htim2,int Input,unsigned int Channel1,unsigned int Channel2){
	motor->htim2 = htim2;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;
	motor->Channel2 = Channel2;

	if(Input<0){
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel1,0);
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel2,motor->Pwm);

	}
	else if(Input>0){
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel1,motor->Pwm);
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel2,0);
	}
	else{
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel1,0);
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel2,0);
	}

}
