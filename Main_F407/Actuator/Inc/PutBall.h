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

typedef struct PutBallParam_t{
	Encoder_t putBall_enc;
	MotorDC putBall_mdc;
	AccelParam_t accel;
	uint8_t flag;
	uint8_t status;
	uint8_t rst;
}PutBallParam_t;

void startPutBall(uint8_t status);

#endif /* INC_PUTBALL_H_ */
