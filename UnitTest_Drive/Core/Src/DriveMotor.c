/*
 * DriveMotor.c
 *
 *  Created on: Sep 5, 2023
 *      Author: defaultuser0
 */
#include "DriveMotor.h"
#include "stdlib.h"

void BLDC_Drive_RedBoard(MotorDrive *motor,TIM_HandleTypeDef *htim1,int Input,unsigned int Channel1)
{
	motor->htim1 = htim1;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;
	if (Input>0)
	{
		HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, 1);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
	}
	else
	{
		HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, 0);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
	}
}

void DC_Drive_BTS(MotorDrive *motor,TIM_HandleTypeDef *htim1,uint16_t Mode,int Input,unsigned int Channel1,unsigned int Channel2)
{
	motor->htim1 = htim1;
	motor->Mode = Mode;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;
	motor->Channel2 = Channel2;

	if(motor->Mode==0){
		if(Input < 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,0);
		}
		else if(Input > 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,0);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Pwm);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,0);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,0);
		}
	}else{
		if(Input < 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Mode-motor->Pwm);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Mode);
		}
		else if(Input > 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Mode);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Mode-motor->Pwm);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Mode);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Mode);
		}
	}
}
