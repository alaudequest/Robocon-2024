/*
 * ActuatorValve.h
 *
 *  Created on: Jan 15, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_RB1ACTUATORVALVE_H_
#define INC_RB1ACTUATORVALVE_H_

#include "main.h"
#include "74HC595.h"

#define VALVE_ARM_LEFT_HC595_PIN 2
#define VALVE_HAND_LEFT_HC595_PIN 4
#define VALVE_COLLECT_BALL_RIGHT_HC595_PIN 0
#define VALVE_COLLECT_BALL_LEFT_HC595_PIN 3
#define VALVE_ARM_RIGHT_HC595_PIN 5
#define VALVE_HAND_RIGHT_HC595_PIN 1

typedef enum ValveProcessName {
	ValveProcess_CatchAndHold = 1,
	ValveProcess_ShootBallTime_Start,
	ValveProcess_ShootBallTime_Stop,
	ValveProcess_ShootBallTime_Reset,
	ValveProcess_ShootBallTime_GetBallLeft,
	ValveProcess_ShootBallTime_GetBallRight,
} ValveProcessName;

void valve_TestPin(pinName pin);
void valve_TestBlinkAll();
void valve_Output(uint8_t outputPort, bool on);
void valve_Init();
void valve_Reset();
void valve_OutputAllPin(uint8_t valveArrayOutput);
void valve_ArmUp();
void valve_ArmDown();
void valve_ProcessBegin(ValveProcessName _valveProcessName);
void valve_HandHold();
void valve_HandRelease();
void valve_OpenLeftCollectBall();
void valve_CloseLeftCollectBall();
void valve_OpenRightCollectBall();
void valve_CloseRightCollectBall();
void RB1_valve_ProcessManager();
bool valve_IsProcessEnd();
#endif /* INC_RB1ACTUATORVALVE_H_ */
