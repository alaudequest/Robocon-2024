/*
 * PID_SwerveModule.c
 *
 *  Created on: Oct 3, 2023
 *      Author: KHOA
 */

#include "PID_SwerveModule.h"


void PID_DC_CalSpeed(float Target_set)
{
	MotorDC mdc = brd_GetObjMotorDC();
	Encoder_t encDC = brd_GetObjEncDC();
	PID_Param pid = brd_GetPID(PID_DC_SPEED);
	PID_Cal(&pid, Target_set, encoder_GetSpeed(&encDC));
	MotorDC_Drive(&mdc, (int32_t)pid.u);
	brd_SetObjEncDC(encDC);
	brd_SetPID(pid, PID_DC_SPEED);
}

void PID_DC_CalPos(float Target_set)
{
	Encoder_t encDC = brd_GetObjEncDC();
	PID_Param pid = brd_GetPID(PID_DC_ANGLE);
	PID_Cal(&pid, Target_set, encoder_GetPulse(&encDC, MODE_ANGLE));
	PID_DC_CalSpeed(pid.u);
	//brd_SetObjEncDC(encDC);
	brd_SetPID(pid, PID_DC_ANGLE);
}

void PID_BLDC_CalSpeed(float Target_set)
{
	MotorBLDC mbldc = brd_GetObjMotorBLDC();
	Encoder_t encBLDC = brd_GetObjEncBLDC();
	PID_Param pid = brd_GetPID(PID_BLDC_SPEED);
	PID_Cal(&pid, Target_set, encoder_GetSpeed(&encBLDC));
	MotorBLDC_Drive(&mbldc, (int32_t)pid.u);
	brd_SetObjEncBLDC(encBLDC);
	brd_SetPID(pid, PID_BLDC_SPEED);
}


