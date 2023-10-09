/*
 * SetHome.c
 *
 *  Created on: Oct 4, 2023
 *      Author: KHOA
 */

#include "SetHome.h"
#include "Encoder.h"
#include "BoardParameter.h"
#include "PID_SwerveModule.h"
#include "cmsis_os.h"
SetHomeEvent homeEvent = 0;
float speed = 0;

void sethome_Begin()
{
	homeEvent = SET_HOME_BEGIN;
}

void homeBeginHandle(){
	if(!HAL_GPIO_ReadPin(Sensor_Home_GPIO_Port, Sensor_Home_Pin)){
		HAL_Delay(1);
		if(!HAL_GPIO_ReadPin(Sensor_Home_GPIO_Port, Sensor_Home_Pin)){
			homeEvent = SET_HOME_COMPLETE;
		}
	} else {
		speed = TUNE_COARSE_SPEED;
		homeEvent = SET_HOME_TUNE_COARSE;
	}
}

void tuneCoarseHandle(){
	if(brd_GetAngleDC_Fast() > TUNE_COARSE_ABOVE_DEGREE && speed > 0){
		speed *= -1;
	} else if (brd_GetAngleDC_Fast() < TUNE_COARSE_BELOW_DEGREE && speed < 0){
		speed *= -1;
	}
	if(!HAL_GPIO_ReadPin(Sensor_Home_GPIO_Port, Sensor_Home_Pin)){
		osDelay(1);
		if(!HAL_GPIO_ReadPin(Sensor_Home_GPIO_Port, Sensor_Home_Pin)){
			speed = TUNE_FINE_SPEED;
			homeEvent = SET_HOME_TUNE_COARSE_SENSOR_DETECT;
		}
	}
}
void tuneFineHandle(){
	if(brd_GetAngleDC_Fast() > TUNE_FINE_ABOVE_DEGREE && speed > 0){
		speed *= -1;
	} else if (brd_GetAngleDC_Fast() < TUNE_FINE_BELOW_DEGREE && speed < 0){
		speed *= -1;
	}
	if(!HAL_GPIO_ReadPin(Sensor_Home_GPIO_Port, Sensor_Home_Pin)){
		HAL_Delay(10);
		if(!HAL_GPIO_ReadPin(Sensor_Home_GPIO_Port, Sensor_Home_Pin)){
			speed = 0;
			homeEvent = SET_HOME_TUNE_FINE_SENSOR_DETECT;
		}
	}
}



bool sethome_IsComplete()
{
	if(homeEvent == SET_HOME_COMPLETE) return 1;
	return 0;
}

void sethome_Procedure()
{
	switch(homeEvent){
	case SET_HOME_BEGIN:
		homeBeginHandle();
		break;
	case SET_HOME_TUNE_COARSE:
		tuneCoarseHandle();
		break;
	case SET_HOME_TUNE_COARSE_SENSOR_DETECT:
		brd_ResetState();
		homeEvent = SET_HOME_TUNE_FINE;
		break;
	case SET_HOME_TUNE_FINE:
		tuneFineHandle();
		break;
	case SET_HOME_TUNE_FINE_SENSOR_DETECT:
		brd_ResetState();
		homeEvent = SET_HOME_STEADY;
		break;
	case SET_HOME_STEADY:
		homeEvent = SET_HOME_COMPLETE;
		break;
	case SET_HOME_COMPLETE:

		break;
	}

}

float sethome_GetSpeed(){return speed;}

#define TARGET_FLAG_GROUP homeEvent
void sethome_SetFlag(SetHomeEvent e){SETFLAG(TARGET_FLAG_GROUP,e);}
bool sethome_CheckFlag(SetHomeEvent e){return CHECKFLAG(TARGET_FLAG_GROUP,e);}
void sethome_ClearFlag(SetHomeEvent e){CLEARFLAG(TARGET_FLAG_GROUP,e);}
