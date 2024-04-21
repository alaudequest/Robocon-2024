/*
 * PutBall.h
 *
 *  Created on: Mar 16, 2024
 *      Author: khoac
 */

#ifndef INC_PUTBALL_H_
#define INC_PUTBALL_H_

#include "main.h"
#include "Encoder.h"
#include "Motor.h"
#include "Accel.h"

typedef struct PutParam_t{
	Encoder_t enc;
	MotorDC mdc;
	AccelParam_t accel;
	uint8_t flag;
	uint8_t status;
	uint8_t StopPutFlag;
	int count;
	int putBall_SScheck;
}PutParam_t;

typedef struct GetParam_t{
	Encoder_t enc;
	MotorDC mdc;
	AccelParam_t accel;
	uint8_t flag;
	uint8_t status;
}GetParam_t;

void startPutBall(uint8_t status);
uint8_t PutBall_getFlag();
uint8_t putBall_getStopFlag();
#endif /* INC_PUTBALL_H_ */
