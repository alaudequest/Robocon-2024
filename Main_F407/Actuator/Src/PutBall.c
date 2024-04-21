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
		putBall.putBall_SScheck = 0;
		putBall.count += 1;
		putBall.StopPutFlag = 0;
		if(putBall.count < 30*50)
		{
			MotorDC_Drive(&putBall.mdc, -200);
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
		putBall.flag = 0;
		putBall.accel.vel_controller = 0;
		getBall.accel.vel_controller = 0;
		if(putBall.StopPutFlag == 0)
		{
			if(HAL_GPIO_ReadPin(sensor_5_GPIO_Port, sensor_5_Pin))
			{
				putBall.putBall_SScheck += 1;
			}else{
				putBall.putBall_SScheck = 0;
			}
			if(putBall.putBall_SScheck > 5){
				putBall.StopPutFlag = 1;
				putBall.putBall_SScheck = 0;
			}

		}
		if (putBall.StopPutFlag)
		{
			MotorDC_Drive(&putBall.mdc, 100);
			MotorDC_Drive(&getBall.mdc, 0);
			encoder_ResetCount(&putBall.enc);
		}else{
			MotorDC_Drive(&putBall.mdc, 400);
			MotorDC_Drive(&getBall.mdc, -1000);
		}
	}
	else if(state == 2)
	{
		putBall.putBall_SScheck = 0;
		putBall.StopPutFlag = 0;
		putBall.count += 1;

		if (putBall.count<30*50)
		{
			MotorDC_Drive(&putBall.mdc,450);
			MotorDC_Drive(&getBall.mdc, -1000);
		}else {
			MotorDC_Drive(&putBall.mdc,100);
			MotorDC_Drive(&getBall.mdc, -1000);
		}
	}
	else if(state == 3)
	{
		putBall.count = 0;
		putBall.flag = 0;
		putBall.accel.vel_controller = 0;
		getBall.accel.vel_controller = 0;

		if(putBall.StopPutFlag == 0)
		{
			if(HAL_GPIO_ReadPin(sensor_5_GPIO_Port, sensor_5_Pin))
			{
				putBall.putBall_SScheck += 1;
			}else{
				putBall.putBall_SScheck = 0;
			}
			if(putBall.putBall_SScheck > 5){
				putBall.StopPutFlag = 1;
				putBall.putBall_SScheck = 0;
			}

		}
		if (putBall.StopPutFlag)
		{
			MotorDC_Drive(&putBall.mdc, 100);
			MotorDC_Drive(&getBall.mdc, 0);
			encoder_ResetCount(&putBall.enc);
		}else{
			MotorDC_Drive(&putBall.mdc, -150);
			MotorDC_Drive(&getBall.mdc, -1000);
		}
	}else if(state == 4)
	{
		putBall.putBall_SScheck = 0;
		putBall.count += 1;
		putBall.StopPutFlag = 0;
		if(putBall.count < 30*50)
		{
			MotorDC_Drive(&putBall.mdc, -200);
			putBall.flag = 0;
		}
		else
		{
			MotorDC_Drive(&putBall.mdc, 0);
			putBall.flag = 1;
		}
		MotorDC_Drive(&getBall.mdc, 0);
	}
}

uint8_t PutBall_getFlag()
{
	return putBall.flag;
}

uint8_t putBall_getStopFlag()
{
	return putBall.StopPutFlag;
}
