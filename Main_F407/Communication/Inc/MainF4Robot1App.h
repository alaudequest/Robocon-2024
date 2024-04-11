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
#include "BoardParameter.h"
#include "PID.h"

typedef enum MainF4Robot1RelayCommand {
	RelayCmd_RunRuloCollectBall,
	RelayCmd_RunRuloShootBall,
} MainF4Robot1RelayCommand;

typedef struct AppPararmPID_t {
	MainF4Robot1TypePID typePID;
	float kp;
	float ki;
	float kd;
	float alpha;
	float deltaT;
	float limitHigh;
	float limitLow;
} AppPararmPID_t;
void MainF4Robot1App_Init();

#endif /* INC_MAINF4ROBOT1APP_H_ */
