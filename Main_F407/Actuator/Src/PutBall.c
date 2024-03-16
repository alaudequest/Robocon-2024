/*
 * PutBall.c
 *
 *  Created on: Mar 16, 2024
 *      Author: khoac
 */

#include "PutBall.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void startPutBall(uint8_t status)
{
	PutBallParam_t putBall;
	MotorDC_Init(&putBall.putBall_mdc, &htim3, MOTOR_PWM_INVERSE, TIM_CHANNEL_1, TIM_CHANNEL_2);
	encoder_Init(&putBall.putBall_enc, &htim2, 19200, 0.001);
	if(status == 0)
	{
		putBall.status = 0;
		MotorDC_Drive(&putBall.putBall_mdc, -200);
	}
	if(status == 1)
	{
		putBall.status = 1;
		if(HAL_GPIO_ReadPin(SSPutBall_GPIO_Port, SSPutBall_Pin)!= 1 )
		{

			MotorDC_Drive(&putBall.putBall_mdc, 100);

		}else{
			MotorDC_Drive(&putBall.putBall_mdc, 0);
		}
	}
	if(status == 2)
	{
		putBall.status = 2;
		MotorDC_Drive(&putBall.putBall_mdc, 200);
	}
}
