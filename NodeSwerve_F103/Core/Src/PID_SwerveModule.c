/*
 * PID_SwerveModule.c
 *
 *  Created on: Oct 3, 2023
 *      Author: KHOA
 */

#include "PID_SwerveModule.h"
bool bldcEnablePID = false;
bool dcEnablePID = true;
float SaiSoChoPhep;
float TocDoBLDC;
float uHatTruocDo;


void PID_DC_CalSpeed(float Target_set)
{
	MotorDC mdc = brd_GetObjMotorDC();
	Encoder_t encDC = brd_GetObjEncDC();
	PID_Param pid = brd_GetPID(PID_DC_SPEED);
	if(dcEnablePID){
		float result = PID_Cal(&pid, Target_set, encoder_GetSpeed(&encDC));
		brd_SetPID(pid, PID_DC_SPEED);
		brd_SetObjEncDC(encDC);
		MotorDC_Drive(&mdc, (int32_t)result);
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
	if(!bldcEnablePID){
		MotorBLDC_Drive(&mbldc, 0);
	}else{
//		float result = Target_set;
		TocDoBLDC = encoder_GetSpeed(&encBLDC);
		float result = PID_Cal(&pid, Target_set, encBLDC.deltaXung);
		if ((pid.e>-SaiSoChoPhep)&&(pid.e<SaiSoChoPhep))
		{

			pid.uHat = uHatTruocDo;
			result=uHatTruocDo;
		}
		uHatTruocDo = pid.uHat;
		MotorBLDC_Drive(&mbldc, (int32_t)result);
		brd_SetObjEncBLDC(encBLDC);
		brd_SetPID(pid, PID_BLDC_SPEED);
	}
}
void PID_ALL_Enable(bool Mode)
{
	bldcEnablePID = Mode;
	dcEnablePID = Mode;
}
void PID_BLDC_BreakProtection(bool Mode)
{
	if(Mode) {
		bldcEnablePID = false;
		MotorBLDC mbldc = brd_GetObjMotorBLDC();
		PID_Param pid = brd_GetPID(PID_BLDC_SPEED);
		pid.uI = 0;
		brd_SetPID(pid, PID_BLDC_SPEED);
		MotorBLDC_Drive(&mbldc, 0);
		return;
	}else bldcEnablePID = true;
}

