/*
 * BoardParameter.h
 *
 *  Created on: Sep 23, 2023
 *      Author: SpiritBoi
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_
#include "main.h"
#include "PID.h"
#include "stdbool.h"
#include "InverseKinematic.h"



typedef enum MainF4Robot1TypePID {
	PID_RULO_1 = 1,
	PID_RULO_2,
	PID_ROBOT_THETA,
} MainF4Robot1TypePID;

typedef enum SignalButtonColor{
	SIGBTN_RED = 1,
	SIGBTN_YELLOW,
	SIGBTN_BLUE,
	SIGBTN_GREEN,
}SignalButtonColor;
typedef void (*pSignalBtnPressed)(SignalButtonColor);

void ProcessDelay(uint32_t delayMs, uint8_t *step);
HAL_StatusTypeDef BuzzerBeep_Start(uint8_t repeatTime, uint32_t onDelayMs,uint32_t offDelayMs);
void BuzzerBeepProcess();
void RobotSignalButton_ScanButton();
#endif /* INC_BOARDPARAMETER_H_ */
