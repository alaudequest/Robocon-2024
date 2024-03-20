/*
 * Robot2_BallTransferToSilo.h
 *
 *  Created on: Mar 20, 2024
 *      Author: KHOA
 */

#ifndef INC_ROBOT2_BALLTRANSFERTOSILO_H_
#define INC_ROBOT2_BALLTRANSFERTOSILO_H_

#include "main.h"
#include "Encoder.h"
#include "Motor.h"
#include "Accel.h"

typedef struct ForkliftParam_t {
	Encoder_t enc;
	MotorDC mdc;
	AccelerationController_t accel;
} ForkliftParam_t;

typedef struct RuloParam_t {
	MotorDC mdc;
} RuloParam_t;

void balltransfer_Init();
void balltransfer_Rulo_Activate(bool activate);
void balltransfer_Forklift_Move(float speed);

#endif /* INC_ROBOT2_BALLTRANSFERTOSILO_H_ */
