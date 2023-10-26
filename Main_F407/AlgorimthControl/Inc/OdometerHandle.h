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

#define ODO_WHEEL_RADIUS 		58
#define ODO_WHEEL_LR_DISTANCE	400
#define ODO_WHEEL_UD_DISTANCE 	355

#define ODO_PULSE_PER_ROUND		200
#define ODO_DeltaT				0.001

typedef struct odometerParam{
	Encoder_t encYR;
	Encoder_t encYL;
	Encoder_t encX;

	float EncCoef;

	float deltaX;
	float deltaY;

	float 	CurrentXPos;
	float	CurrentYPos;
	float 	CurrentTheta;

	float 	DeltaCurrentXPos;
	float	DeltaCurrentYPos;
	float 	DeltaCurrentTheta;
}odometerParam;
void PositionCalculate(void);
void OdoInit(void);
#endif /* INC_ODOMETERHANDLE_H_ */
