/*
 * ActuatorValve.h
 *
 *  Created on: Jan 15, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_ACTUATORVALVE_H_
#define INC_ACTUATORVALVE_H_

#include "main.h"
#include "74HC595.h"

void valve_TestPin(pinName pin);
void valve_TestBlinkAll();
void valve_Output(uint8_t outputPort, bool on);
void valve_Gap1();
void valve_Tha1();
void valve_Gap2();
void valve_Tha2();
void valve_BothCatch();
void valve_BothRelease();
void valve_Init();
#endif /* INC_ACTUATORVALVE_H_ */
