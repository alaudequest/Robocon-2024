/*
 * Motor.c
 *
 *  Created on: Sep 24, 2023
 *      Author: KHOA
 */

#include "Motor.h"
#include <stdbool.h>
#include <stdlib.h>
void MotorDC_Init(MotorDC *dcMotor,TIM_HandleTypeDef *htim, PWM_Mode pwmMode, uint32_t channel1, uint32_t channel2)
{
	dcMotor->Channel1 = channel1;
	dcMotor->Channel2 = channel2;
 	dcMotor->invPWM = pwmMode;
 	dcMotor->timDC = htim;
}
void MotorDC_Drive(MotorDC *dcMotor, int32_t speedInput)
{
	uint16_t invPWM = dcMotor->invPWM;
	uint32_t pwm = abs(speedInput);
	switch(dcMotor->invPWM){
		case MOTOR_PWM_NORMAL:
			__HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel2,0);
			__HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel1,0);
			if(speedInput < 0)__HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel1,pwm);
			else if(speedInput > 0) __HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel2,pwm);
		break;
		case MOTOR_PWM_INVERSE:
			__HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel2,invPWM);
			__HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel1,invPWM);
			if(speedInput < 0)__HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel1,invPWM - pwm);
			else if(speedInput > 0) __HAL_TIM_SET_COMPARE(dcMotor->timDC,dcMotor->Channel2,invPWM - pwm);
		break;
	}
}
void MotorBLDC_Init(MotorBLDC *bldcMotor,TIM_HandleTypeDef *htim, uint32_t channel,
					GPIO_TypeDef *brakePort, uint16_t brakePin,
					GPIO_TypeDef *dirPort, uint16_t dirPin)
{
	bldcMotor->Channel = channel;
	bldcMotor->timBLDC = htim;
	bldcMotor->brakePin = brakePin;
	bldcMotor->brakePort = brakePort;
	bldcMotor->dirPort = dirPort;
	bldcMotor->dirPin = dirPin;
}
void MotorBLDC_Brake(MotorBLDC *bldcMotor,bool brake)
{
	HAL_GPIO_WritePin(bldcMotor->brakePort, bldcMotor->brakePin, brake);
}

void MotorBLDC_Drive(MotorBLDC *bldcMotor, int32_t speedInput)
{
	__HAL_TIM_SET_COMPARE(bldcMotor->timBLDC,bldcMotor->Channel, 0);
	if(!speedInput) return;
	uint32_t pwm = abs(speedInput);
	if(speedInput < 0) 			HAL_GPIO_WritePin(bldcMotor->dirPort, bldcMotor->dirPin, DIR_REVERSE);
	else if(speedInput > 0) 	HAL_GPIO_WritePin(bldcMotor->dirPort, bldcMotor->dirPin, DIR_FORWARD);
	__HAL_TIM_SET_COMPARE(bldcMotor->timBLDC,bldcMotor->Channel, pwm);
}
