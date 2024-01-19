/*
 * DriveMotor.h
 *
 *  Created on: Sep 5, 2023
 *      Author: defaultuser0
 */

#ifndef INC_DRIVEMOTOR_H_
#define INC_DRIVEMOTOR_H_

#include "main.h"

typedef struct MotorDrive{
	TIM_HandleTypeDef *htim1;
	TIM_HandleTypeDef *htim2;
	int Input;
	int8_t Dir;
	uint16_t Pwm;
	unsigned int Channel1;
}MotorDrive;


void DC_Drive(MotorDrive *motor,TIM_HandleTypeDef *htim1,int Input,unsigned int Channel1);


#endif /* INC_DRIVEMOTOR_H_ */
