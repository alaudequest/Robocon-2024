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

void valve_TestPin(pinName pin);
void valve_TestBlinkAll();
void valve_Output(uint8_t outputPort, bool on);
void valve_BothCatch();
void valve_BothRelease();
void valve_Init();
void valve_BothHold();
void valve_OutputAllPin(uint8_t valveArrayOutput);
void valve_LeftCollectBall();
void valve_LeftWaitCollectBall();
void valve_RightCollectBall();
void valve_RightWaitCollectBall();
void valve_ArmDownAndHandHold();
void valve_ArmUp();
void valve_HandRelease();
void valve_Reset();
#endif /* INC_RB1ACTUATORVALVE_H_ */