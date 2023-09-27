/*
 * Motor.h
 *
 *  Created on: Sep 24, 2023
 *      Author: KHOA
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "stdbool.h"
typedef enum MotorDirection{
	DIR_FORWARD,
	DIR_REVERSE,
}MotorDirection;

typedef enum PWM_Mode{
	MOTOR_PWM_NORMAL = 0,
	MOTOR_PWM_INVERSE = 1000,
}PWM_Mode;

typedef struct MotorDC{
	TIM_HandleTypeDef *timDC;
	uint16_t invPWM;
	unsigned int Channel1;
	unsigned int Channel2;
}MotorDC;

typedef struct MotorBLDC{
	unsigned int Channel;
	uint16_t brakePin;
	uint16_t dirPin;
	GPIO_TypeDef *brakePort;
	GPIO_TypeDef *dirPort;
	TIM_HandleTypeDef *timBLDC;
}MotorBLDC;

void MotorDC_Init(MotorDC *dcMotor,TIM_HandleTypeDef *htim, PWM_Mode pwmMode, uint32_t channel1, uint32_t channel2);
void MotorDC_Drive(MotorDC *dcMotor, int32_t speedInput);
void MotorBLDC_Init(MotorBLDC *bldcMotor,TIM_HandleTypeDef *htim, uint32_t channel,
					GPIO_TypeDef *brakePort, uint16_t brakePin,
					GPIO_TypeDef *dirPort, uint16_t dirPin);
void MotorBLDC_Brake(MotorBLDC *bldcMotor,bool brake);
void MotorBLDC_Drive(MotorBLDC *bldcMotor, int32_t speedInput);
#endif /* INC_MOTOR_H_ */
