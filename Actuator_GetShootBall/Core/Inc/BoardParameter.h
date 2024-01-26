/*
 * BoardParameter.h
 *
 *  Created on: Jan 18, 2024
 *      Author: NamDHay
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_

#include "main.h"
#include "PID.h"
#include "Flag.h"
#include "Motor.h"
#include "Encoder.h"
#include "CAN_Control.h"
#include "SetHome.h"

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
#define RotaryEncoderPerRound 				  1000
#define RotaryGearRatio 					  3.535
#define RotaryDeltaT 						  0.001
/*-----------------------------End:Rotary Macro-------------------------------*/

#define DC_SUM_ABOVE_LIMIT 						1000
#define DC_SUM_BELOW_LIMIT 						-1000

#define PIDDeltaT							0.001



typedef enum PID_type{
	PID_GUN1 = 1,
	PID_GUN2,
	PID_ROTARY_ANGLE,
	PID_ROTARY_SPEED,
	PID_BLDC_SPEED,
	PID_DC_ANGLE,
	PID_DC_SPEED
}PID_type;

typedef struct BoardParameter_t {
	float targetAngleDC;
	float targetSpeedGun1;
	float targetSpeedGun2;
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
	PID_Param pidRotaryAngle;
	PID_Param pidRotarySpeed;
}BoardParameter_t;

typedef enum Motor_Type{
	MOTOR_GUN1,
	MOTOR_GUN2,
	MOTOR_BALL1,
	MOTOR_BALL2,
	MOTOR_ROTARY,
}Motor_Type;

void brd_SetHomeCompleteCallback();

void brd_Init();

void brd_SetPID(PID_Param pid,PID_type type);
PID_Param brd_GetPID(PID_type type);

Motor brd_GetObjMotor(Motor_Type type);
void brd_SetObjMotor(Motor motor, Motor_Type type);

void brd_SetObjEncGun(Encoder_t encGun, Motor_Type gun);
Encoder_t brd_GetObjEncGun(Motor_Type gun);

float brd_GetTargetRotaryAngle();
void brd_SetTargetRotaryAngle(float angle);

float brd_GetSpeedGun(Motor_Type type);
void brd_SetSpeedGun(float speed, Motor_Type type);

float brd_GetDeltaT();
void brd_SetDeltaT(float deltaT);

void brd_SetObjEncRotary(Encoder_t encBLDC);
Encoder_t brd_GetObjEncRotary();

void brd_ResetState();

#endif /* INC_BOARDPARAMETER_H_ */
