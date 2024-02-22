/*
 * PID_GunModule.c
 *
 *  Created on: Jan 21, 2024
 *      Author: namdhay
 */

#include "PID_GunModule.h"

void PID_Gun_CalSpeed(float Target_set, Motor_Type gun){
	Motor mdc = brd_GetObjMotor(gun);
	Encoder_t edc = brd_GetObjEncGun(gun);
	PID_type pidtype;
	if(gun == MOTOR_GUN1)	pidtype = PID_GUN1;
	else					pidtype = PID_GUN2;
	PID_Param pid = brd_GetPID(pidtype);
	float result = PID_Cal(&pid, Target_set, encoder_GetSpeed(&edc));
	Motor_Drive(&mdc, (int32_t)result);
	brd_SetObjEncGun(edc, gun);
	brd_SetPID(pid, pidtype);
}

void PID_RuloBall_CalSpeed(float Target_set, Motor_Type rulo){
	Motor mdc = brd_GetObjMotor(rulo);
	Motor_Drive(&mdc, Target_set);
}

void PID_Motor_Stop_All(){
	PID_RuloBall_CalSpeed(0, MOTOR_BALL1);
	PID_RuloBall_CalSpeed(0, MOTOR_BALL2);
	PID_Gun_CalSpeed(0, MOTOR_GUN1);
	PID_Gun_CalSpeed(0, MOTOR_GUN2);
}
