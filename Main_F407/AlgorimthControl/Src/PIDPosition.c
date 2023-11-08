/*
 * PIDPosition.c
 *
 *  Created on: Nov 1, 2023
 *      Author: Admin
 */

#include "PIDPosition.h"

PID_POS	pidP;

void PID_Pos_Init(){
	pidP.pidPos_x.kP = 0.015;
	pidP.pidPos_x.kI = 0;
	pidP.pidPos_x.kD = 0;
	pidP.pidPos_x.kB = 0;
	pidP.pidPos_x.alpha = 0;
	pidP.pidPos_x.deltaT = posDeltaT;
	pidP.pidPos_x.u_AboveLimit = 0.3;
	pidP.pidPos_x.u_BelowLimit = -0.3;

	pidP.pidPos_y.kP = 0.015;
	pidP.pidPos_y.kI = 0;
	pidP.pidPos_y.kD = 0;
	pidP.pidPos_y.kB = 0;
	pidP.pidPos_y.alpha = 0;
	pidP.pidPos_y.deltaT = posDeltaT;
	pidP.pidPos_y.u_AboveLimit = 0.3;
	pidP.pidPos_y.u_BelowLimit = -0.3;

	pidP.pidPos_theta.kP = 0.015;
	pidP.pidPos_theta.kI = 0;
	pidP.pidPos_theta.kD = 0;
	pidP.pidPos_theta.kB = 0;
	pidP.pidPos_theta.alpha = 0;
	pidP.pidPos_theta.deltaT = posDeltaT;
	pidP.pidPos_theta.u_AboveLimit = 0.3;
	pidP.pidPos_theta.u_BelowLimit = -0.3;

	pidP.pidPos_x_fil.filAlpha = 0;
	pidP.pidPos_y_fil.filAlpha = 0;
	pidP.pidPos_theta_fil.filAlpha = 0;
}

void PID_CalPos_SetPID(PID_Param pid,PID_POS_type type)
{
	switch(type){
		case PID_POS_X:
			pidP.pidPos_x = pid;
			break;
		case PID_POS_Y:
			pidP.pidPos_y = pid;
			break;
		case PID_POS_THETA:
			pidP.pidPos_theta = pid;
			break;
		}
}

void PID_CalPos_Setfil(lowPassParam fil,PID_POS_type type)
{
	switch(type){
		case PID_POS_X:
			pidP.pidPos_x_fil = fil;
			break;
		case PID_POS_Y:
			pidP.pidPos_y_fil = fil;
			break;
		case PID_POS_THETA:
			pidP.pidPos_theta_fil = fil;
			break;
		}
}

PID_Param PID_CalPos_GetPID(PID_POS_type type)
{
	switch(type){
	case PID_POS_X:
		return pidP.pidPos_x;
		break;
	case PID_POS_Y:
		return pidP.pidPos_y;
		break;
	case PID_POS_THETA:
		return pidP.pidPos_theta;
		break;
	}
	return pidP.pidPos_x;
}

lowPassParam PID_CalPos_Getfil(PID_POS_type type)
{
	switch(type){
	case PID_POS_X:
		return pidP.pidPos_x_fil;
		break;
	case PID_POS_Y:
		return pidP.pidPos_y_fil;
		break;
	case PID_POS_THETA:
		return pidP.pidPos_theta_fil;
		break;
	}
	return pidP.pidPos_x_fil;
}

float PID_CalPos_x(float Target_set)
{
	PID_Param pid = PID_CalPos_GetPID(PID_POS_X);
	float result = PID_Cal(&pid, Target_set, Odo_GetPulseX()*M_PI*ODO_WHEEL_RADIUS*1/(ODO_PULSE_PER_ROUND*4));
	PID_CalPos_SetPID(pid, PID_POS_X);
	return result;
}

float PID_CalPos_y(float Target_set)
{
	PID_Param pid = PID_CalPos_GetPID(PID_POS_Y);
	float result = PID_Cal(&pid, Target_set, Odo_GetPulseYR()*M_PI*ODO_WHEEL_RADIUS*1/(ODO_PULSE_PER_ROUND*4));
	PID_CalPos_SetPID(pid, PID_POS_Y);
	return result;
}

float PID_CalPos_theta(float Target_set)
{
	PID_Param pid = PID_CalPos_GetPID(PID_POS_THETA);
	float result = PID_Cal(&pid, Target_set, Odo_GetPOS_Theta());
	PID_CalPos_SetPID(pid, PID_POS_THETA);
	return result;
}
