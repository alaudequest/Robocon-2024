/*
 * DriveMotor.h
 *
 *  Created on: Sep 5, 2023
 *      Author: defaultuser0
 */

#ifndef INC_DRIVEMOTOR_H_
#define INC_DRIVEMOTOR_H_

#include "main.h"

#define motor_Reserve 1000
#define motor_Normal 1

typedef struct MotorDrive{
	TIM_HandleTypeDef *htim1;
	TIM_HandleTypeDef *htim2;
	int Input;
	int8_t Dir;
	uint16_t Pwm;
	uint16_t Mode;
	unsigned int Channel1;
	unsigned int Channel2;
}MotorDrive;


void DC_Drive_BTS(MotorDrive *motor,TIM_HandleTypeDef *htim1,uint16_t Mode,int Input,unsigned int Channel1,unsigned int Channel2);
//void BLDC_Drive_RedBoard(MotorDrive *motor,TIM_HandleTypeDef *htim1,int Input,unsigned int Channel1);


#endif /* INC_DRIVEMOTOR_H_ */
