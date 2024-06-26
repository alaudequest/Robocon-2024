/*
 * BoardParameter.h
 *
 *  Created on: Sep 23, 2023
 *      Author: KHOA
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_
#include "main.h"
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
/*-----------------------------Begin:BLDC Macro-------------------------------*/
#define _BLDCEncoderPerRound 				  200
#define _BLDCGearRatio 						  2.56
#define _BLDCDeltaT							  0.001
/*-----------------------------End:BLDC Macro---------------------------------*/

/*-----------------------------Begin:DC Macro---------------------------------*/
#define DCDeltaT 							  0.001
#define DCEncoderPerRound 					  1000
#define DCGearRatio 						  3.535
/*-----------------------------End:DC Macro-----------------------------------*/

#define DC_SUM_ABOVE_LIMIT 						1000
#define DC_SUM_BELOW_LIMIT 						-1000

#define PIDDeltaT							0.005


typedef enum PID_type{
	PID_DC_SPEED = 1,
	PID_DC_ANGLE,
	PID_BLDC_SPEED,
}PID_type;

typedef struct Safety_Check{
	int EncCheckForDC;
	int EncCheckForBLDC;

	float ValueNow;
	float ValuePre;

	float TargetPre;
	float TargetNow;

	float SaftyFlag;

}Safety_Check;
typedef struct BoardParameter_t {
	float targetAngleDC;
	float targetSpeedBLDC;

	Safety_Check SafeDC;
	Safety_Check SafeBLDC;

	int16_t countTimer;
	MotorDC mdc;
	MotorBLDC mbldc;
	Encoder_t encBLDC;
	Encoder_t encDC;
	PID_Param pidDC_Angle;
	PID_Param pidDC_Speed;
	PID_Param pidBLDC_Speed;
}BoardParameter_t;

void brd_Init();

Safety_Check brd_GetSafyDC();
void brd_SetSafyDC(Safety_Check safe);

Safety_Check brd_GetSafyBLDC();
void brd_SetSafyBLDC(Safety_Check safe);

PID_Param brd_GetPID(PID_type type);
void brd_SetPID(PID_Param pid,PID_type type);

MotorDC brd_GetObjMotorDC();
void brd_SetObjMotorDC(MotorDC mdc);

MotorBLDC brd_GetObjMotorBLDC();
void brd_SetObjMotorBLDC(MotorBLDC mbldc);

Encoder_t brd_GetObjEncDC();
void brd_SetObjEncDC(Encoder_t encDC);

Encoder_t brd_GetObjEncBLDC();
void brd_SetObjEncBLDC(Encoder_t encBLDC);


int32_t brd_GetEncX4BLDC();
void brd_SetEncX4BLDC(int32_t countX4);

float brd_GetTargetSpeedBLDC();
void brd_SetTargetSpeedBLDC(float speed);

float brd_GetTargetAngleDC();
void brd_SetTargetAngleDC(float angle);

float brd_GetCurrentAngleDC();
float brd_GetCurrentSpeedBLDC();
int brd_GetCurrentCountBLDC();

void brd_ResetState();
void brd_SetHomeCompleteCallback();

float brd_GetDeltaT();
void brd_SetDeltaT(float deltaT);



#endif /* INC_BOARDPARAMETER_H_ */
