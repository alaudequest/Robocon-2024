/*
 * SetHome.h
 *
 *  Created on: Jan 21, 2024
 *      Author: namdhay
 */

#ifndef INC_SETHOME_H_
#define INC_SETHOME_H_

#include "main.h"
#include "Flag.h"
#include "stdbool.h"

#define TUNE_COARSE_ABOVE_DEGREE 180
#define TUNE_COARSE_BELOW_DEGREE -180
#define TUNE_COARSE_SPEED 20

#define TUNE_FINE_ABOVE_DEGREE 1
#define TUNE_FINE_BELOW_DEGREE -1
#define TUNE_FINE_SPEED 5

typedef enum SetHomeEvent {
	SET_HOME_BEGIN = 1,
	SET_HOME_TUNE_COARSE,
	SET_HOME_TUNE_COARSE_SENSOR_DETECT,
	SET_HOME_TUNE_FINE,
	SET_HOME_TUNE_FINE_SENSOR_DETECT,
	SET_HOME_STEADY,
	SET_HOME_COMPLETE,
} SetHomeEvent;

float sethome_GetSpeed();
void sethome_Begin();
void sethome_Procedure();
void sethome_SetFlag(SetHomeEvent e);
bool sethome_CheckFlag(SetHomeEvent e);
void sethome_ClearFlag(SetHomeEvent e);
bool sethome_IsComplete();

#endif /* INC_SETHOME_H_ */
