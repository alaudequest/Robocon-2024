/*
 * OdometerHandle.h
 *
 *  Created on: Oct 25, 2023
 *      Author: Admin
 */

#ifndef INC_ODOMETERHANDLE_H_
#define INC_ODOMETERHANDLE_H_
#include "main.h"
#include "Encoder.h"
#include "math.h"

#define ODO_WHEEL_RADIUS 		5.8
#define ODO_WHEEL_LR_DISTANCE	40
#define ODO_WHEEL_UD_DISTANCE 	35.5

#define ODO_PULSE_PER_ROUND		100
#define ODO_DeltaT				0.05

typedef struct odometerParam{
	Encoder_t encYR;
	Encoder_t encYL;
	Encoder_t encX;

	float 	EncCoef;

	float 	deltaX;
	float 	deltaY;

	float 	CurrentXPos;
	float	CurrentYPos;
	float 	CurrentTheta;

	float 	DeltaCurrentXPos;
	float	DeltaCurrentYPos;
	float 	DeltaCurrentTheta;

	float 	DeltaCurrentXPos_Pre;
	float	DeltaCurrentYPos_Pre;
	float 	DeltaCurrentTheta_Pre;
}odometerParam;

void Odo_PosCal(void);
void OdoInit(void);
int Odo_GetPulseYR();
int Odo_GetPulseYL();
int Odo_GetPulseX();

float Odo_GetPOS_X();
float Odo_GetPOS_Y();
float Odo_GetPOS_Theta();

float Odo_GetPOS_XTest();
float Odo_GetPOS_YTest();
float Odo_GetPOS_ThetaTest();

#endif /* INC_ODOMETERHANDLE_H_ */
