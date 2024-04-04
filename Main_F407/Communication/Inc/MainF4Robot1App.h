/*
 * MainF4Robot1App.h
 *
 *  Created on: Apr 4, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_MAINF4ROBOT1APP_H_
#define INC_MAINF4ROBOT1APP_H_

#include "main.h"
#include "AppInterface.h"

typedef enum MainF4Robot1RelayCommand {
	RelayCmd_RunRuloCollectBall1,
	RelayCmd_RunRuloCollectBall2,
	RelayCmd_RunRuloShootBall1,
	RelayCmd_RunRuloShootBall2,
} MainF4Robot1RelayCommand;

typedef struct AppPararmPID_t {
	PID_type type;
	float kp;
	float ki;
	float kd;
	float alpha;
	float deltaT;
	float limitHigh;
	float limitLow;
} AppPararmPID_t;

#endif /* INC_MAINF4ROBOT1APP_H_ */
