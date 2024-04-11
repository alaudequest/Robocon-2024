/*
 * PID_SwerveModule.c
 *
 *  Created on: Oct 3, 2023
 *      Author: KHOA
 */

#include "PID_SwerveModule.h"
bool isBreakProtectionDone = false;

void PID_DC_CalSpeed(float Target_set)
{
	MotorDC mdc = brd_GetObjMotorDC();
	Encoder_t encDC = brd_GetObjEncDC();
	PID_Param pid = brd_GetPID(PID_DC_SPEED);

	Safety_Check safeDC = brd_GetSafyDC();


	float result = PID_Cal(&pid, Target_set, encoder_GetSpeed(&encDC));
	safeDC.ValueNow = encoder_GetPulse(&encDC, MODE_X4);

	if (absf(result)>200)
	{
		if (safeDC.ValueNow == safeDC.ValuePre)
			{
			if(safeDC.SaftyFlag == 0){
				safeDC.EncCheckForDC ++;
			}
			}
		else
			{
				safeDC.EncCheckForDC = 0;
			}
	}

	if (safeDC.EncCheckForDC>=10){
		safeDC.SaftyFlag = 1;
	}
	safeDC.ValuePre = safeDC.ValueNow;

	brd_SetPID(pid, PID_DC_SPEED);
	brd_SetObjEncDC(encDC);
	brd_SetSafyDC(safeDC);

	if (safeDC.SaftyFlag == 0)
	{
		MotorDC_Drive(&mdc, (int32_t) result);
	}else{
		MotorDC_Drive(&mdc, 0);
	}
}

void PID_DC_CalPos(float Target_set)
{
	Encoder_t encDC = brd_GetObjEncDC();
	PID_Param pid = brd_GetPID(PID_DC_ANGLE);
	float result = PID_Cal(&pid, Target_set, encoder_GetPulse(&encDC, MODE_ANGLE));
	brd_SetPID(pid, PID_DC_ANGLE);
	brd_SetObjEncDC(encDC);
	PID_DC_CalSpeed(result);
}

void PID_BLDC_CalSpeed(float Target_set)
{
	MotorBLDC mbldc = brd_GetObjMotorBLDC();
	Encoder_t encBLDC = brd_GetObjEncBLDC();
	PID_Param pid = brd_GetPID(PID_BLDC_SPEED);
	if (isBreakProtectionDone == false) {
		MotorBLDC_Drive(&mbldc, 0);
	}
	else if (Target_set != 0) {
		HAL_GPIO_WritePin(BLDC_BRAKE_GPIO_Port, BLDC_BRAKE_Pin, GPIO_PIN_RESET);
		float result = PID_Cal(&pid, Target_set, encoder_GetSpeed(&encBLDC));
		MotorBLDC_Drive(&mbldc, (int32_t) result);
		brd_SetObjEncBLDC(encBLDC);
		brd_SetPID(pid, PID_BLDC_SPEED);
	}
	else if ((abs(brd_GetCurrentSpeedBLDC()) < 0.5 &&
			abs(encBLDC.vel_Real) < 0.5) || Target_set == 0) {
		HAL_GPIO_WritePin(BLDC_BRAKE_GPIO_Port, BLDC_BRAKE_Pin, GPIO_PIN_SET);
		MotorBLDC_Drive(&mbldc, 0);
		pid.uI = 0;
		pid.e = 0;
		pid.u = 0;
		pid.uHat = 0;
		brd_SetPID(pid, PID_BLDC_SPEED);
		encoder_ResetCount(&encBLDC);
	}

}

void PID_BLDC_BreakProtection(bool Mode)
{
	if (Mode) {
		isBreakProtectionDone = false;
		MotorBLDC mbldc = brd_GetObjMotorBLDC();
		PID_Param pid = brd_GetPID(PID_BLDC_SPEED);
		pid.uI = 0;
		brd_SetPID(pid, PID_BLDC_SPEED);
		MotorBLDC_Drive(&mbldc, 0);
		return;
	}
	else
		isBreakProtectionDone = true;
}

void PID_DC_UntangleWireBLDC()
{
	Encoder_t encDC = brd_GetObjEncDC();
	PID_Param pid = brd_GetPID(PID_DC_ANGLE);
	pid.u_AboveLimit = 20;
	pid.u_BelowLimit = -20;
	float result = PID_Cal(&pid, 0, encoder_GetPulse(&encDC, MODE_ANGLE));
	brd_SetPID(pid, PID_DC_ANGLE);
	brd_SetObjEncDC(encDC);
	PID_DC_CalSpeed(result);
}

void PID_BLDC_OnLowSpeed()
{
	PID_Param pid = brd_GetPID(PID_BLDC_SPEED);
	pid.kP = 0.03;
	pid.kI = 2.5;
	brd_SetPID(pid, PID_BLDC_SPEED);
}
void PID_BLDC_OnHightSpeed()
{
	PID_Param pid = brd_GetPID(PID_BLDC_SPEED);
	pid.kP = 0.03;
	pid.kI = 5;
	brd_SetPID(pid, PID_BLDC_SPEED);
}
