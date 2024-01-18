/*
 * BoardParameter.h
 *
 *  Created on: Jan 18, 2024
 *      Author: NamDHay
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_

#include "main.h"
#include "LibraryConfig.h"
#include "PID.h"
#include "Flag.h"
#include "Motor.h"
#include "Encoder.h"
#include "CAN_Control.h"

/*-----------------------------Begin:Gun1 Macro-------------------------------*/
#define _Gun1EncoderPerRound 				  200
#define _Gun1GearRatio 						  2.56
#define _Gun1DeltaT							  0.001
/*-----------------------------End:Gun1 Macro---------------------------------*/

/*-----------------------------Begin:Gun2 Macro-------------------------------*/
#define _Gun2EncoderPerRound 				  200
#define _Gun2GearRatio 						  2.56
#define _Gun2DeltaT							  0.001
/*-----------------------------End:Gun2 Macro---------------------------------*/

/*-----------------------------Begin:Rotary Macro-----------------------------*/
#define RotaryDeltaT 						  0.001
#define RotaryEncoderPerRound 				  1000
#define RotaryGearRatio 					  3.535
/*-----------------------------End:Rotary Macro-------------------------------*/

#define DC_SUM_ABOVE_LIMIT 						1000
#define DC_SUM_BELOW_LIMIT 						-1000

#define PIDDeltaT							0.001

typedef enum PID_type{
	PID_GUN1 = 1,
	PID_GUN2,
	PID_ANGLE,
}PID_type;

typedef struct BoardParameter_t {
	float targetAngleDC;
	float targetSpeedBLDC;
	int16_t countTimer;

	Motor gun1;
	Motor gun2;
	Motor ball1;
	Motor ball2;
	Motor rotary;

	Encoder_t encRotary;
	Encoder_t encGun1;
	Encoder_t encGun2;

	PID_Param pidGun1;
	PID_Param pidGun2;
	PID_Param pidAngle;
}BoardParameter_t;

void brd_Init();

#endif /* INC_BOARDPARAMETER_H_ */
