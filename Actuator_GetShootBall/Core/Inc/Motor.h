/*
 * Motor.h
 *
 *  Created on: Jan 18, 2024
 *      Author: NamDHay
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "stdlib.h"
#include "stdbool.h"

typedef enum Control_Mode{
	MOTOR_PWM,
	MOTOR_LL,		//LL: Logic Level
}Control_Mode;

typedef struct Motor{
	Control_Mode mode;
	GPIO_TypeDef *port;
	uint16_t pin;
	TIM_HandleTypeDef *timDC;
	unsigned int Channel1;
	unsigned int Channel2;
}Motor;

void Motor_Init(Motor *motor, Control_Mode mode,
				GPIO_TypeDef *port, uint16_t pin,
				TIM_HandleTypeDef *tim, uint32_t channel1, uint32_t channel2);
void Motor_Drive(Motor *motor, int32_t speedInput);

#endif /* INC_MOTOR_H_ */
