/*
 * Motor.c
 *
 *  Created on: Jan 18, 2024
 *      Author: NamDHay
 */

#include "Motor.h"

void Motor_Init(Motor *motor, Control_Mode mode,
				GPIO_TypeDef *port, uint16_t pin,
				TIM_HandleTypeDef *tim, uint32_t channel1, uint32_t channel2)
{
	if(motor -> mode == MOTOR_LL)
	{
		motor -> port = port;
		motor -> pin = pin;
	}else
	{
		motor -> timDC = tim;
		motor -> Channel1 = channel1;
		motor -> Channel2 = channel2;
	}
}

void Motor_Drive(Motor *motor, int32_t speedInput)
{
	uint32_t pwm = abs(speedInput);
	if(motor->mode == MOTOR_LL)
	{
		HAL_GPIO_WritePin(motor->port, motor->pin, (speedInput==0)?0:1);
	}else
	{
		__HAL_TIM_SET_COMPARE(motor->timDC, motor->Channel1, 0);
		__HAL_TIM_SET_COMPARE(motor->timDC, motor->Channel2, 0);
		if(!(motor->Channel2))
		{
			__HAL_TIM_SET_COMPARE(motor->timDC, motor->Channel1, pwm);
		}else
		{
			if(speedInput > 0)
			{
				__HAL_TIM_SET_COMPARE(motor->timDC, motor->Channel1, pwm);
			}else
			{
				__HAL_TIM_SET_COMPARE(motor->timDC, motor->Channel2, pwm);
			}
		}
	}
}
