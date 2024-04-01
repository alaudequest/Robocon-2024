/*
 * NodeSwerve_AppInterface.h
 *
 *  Created on: Mar 30, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_NODESWERVE_APPINTERFACE_H_
#define INC_NODESWERVE_APPINTERFACE_H_

#include "main.h"
#include "AppInterface.h"
#include "BoardParameter.h"

typedef enum NodeSwerveRelayCommand {
	RunMotorBLDC = 1,
	RunMotorDC,
} NodeSwerveRelayCommand;

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

void SwerveApp_Init();


#endif /* INC_NODESWERVE_APPINTERFACE_H_ */
