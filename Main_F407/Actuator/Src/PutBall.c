/*
 * PutBall.c
 *
 *  Created on: Mar 16, 2024
 *      Author: khoac
 */

#include "PutBall.h"
#include "cmsis_os.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
PutParam_t putBall;
GetParam_t getBall;

void startPutBall(uint8_t state)
{

	MotorDC_Init(&putBall.mdc, &htim3, MOTOR_PWM_INVERSE, TIM_CHANNEL_1, TIM_CHANNEL_2);
	MotorDC_Init(&getBall.mdc, &htim5, MOTOR_PWM_NORMAL, TIM_CHANNEL_1, TIM_CHANNEL_2);
	encoder_Init(&putBall.enc, &htim2, 19200, 0.001);
	if(state == 0)
	{
		putBall.count = 0;
		putBall.StopPutFlag = 0;
		if(putBall.accel.vel_controller > -600)
		{
			Accel_Cal(&putBall.accel, -600, 2.25);
			MotorDC_Drive(&putBall.mdc, putBall.accel.vel_controller);
			putBall.flag = 0;
		}
		else
		{
			MotorDC_Drive(&putBall.mdc, 0);
			putBall.flag = 1;
		}
		MotorDC_Drive(&getBall.mdc, -1000);
	}
	else if(state == 1)
	{
		putBall.count = 0;
		putBall.accel.vel_controller = 0;
		getBall.accel.vel_controller = 0;
		if(HAL_GPIO_ReadPin(SSPutBall_GPIO_Port, SSPutBall_Pin))
		{
			osDelay(2);
			if(HAL_GPIO_ReadPin(SSPutBall_GPIO_Port, SSPutBall_Pin)){
				putBall.StopPutFlag = 1;
			}

		}
		if (putBall.StopPutFlag)
		{
			MotorDC_Drive(&putBall.mdc, 0);
			MotorDC_Drive(&getBall.mdc, 0);
			encoder_ResetCount(&putBall.enc);
		}else{
			MotorDC_Drive(&putBall.mdc, 400);
			MotorDC_Drive(&getBall.mdc, -1000);
		}
	}
	else if(state == 2)
	{
		putBall.count += 1;
		if(putBall.count >= 1000)
		{
			putBall.StopPutFlag = 2;
		}
		if (putBall.StopPutFlag == 2)
		{
			Accel_Cal(&putBall.accel, 0, 0.75);
			Accel_Cal(&getBall.accel, 0, 2);
			MotorDC_Drive(&putBall.mdc, putBall.accel.vel_controller);
			MotorDC_Drive(&getBall.mdc, getBall.accel.vel_controller);
		}
		else
		{
			Accel_Cal(&putBall.accel, 450, 1);
			Accel_Cal(&getBall.accel, -1000, 0.5);
			MotorDC_Drive(&putBall.mdc, putBall.accel.vel_controller);
			MotorDC_Drive(&getBall.mdc, getBall.accel.vel_controller);
		}
	}
}

uint8_t PutBall_getFlag()
{
	return putBall.flag;
}

