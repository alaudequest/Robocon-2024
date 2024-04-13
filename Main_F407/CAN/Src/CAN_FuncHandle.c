/*
 * CAN_FuncHandle.c
 *
 *  Created on: Sep 22, 2023
 *      Author: KHOA
 */

#include "CAN_Control.h"
#include "CAN_FuncHandle.h"
#include "BoardParameter.h"
#include "PID.h"
#include <string.h>
#include <stdbool.h>

void canfunc_HandleRxEvent(void(*pCallback)(CAN_MODE_ID ID))
{
	uint32_t ce = canctrl_GetEvent();
	if(!ce || !pCallback) return;
	for(uint8_t i = CANCTRL_MODE_START + 1; i < CANCTRL_MODE_END; i++){
		if(canctrl_CheckFlag(i)) {
			pCallback(i);
			canctrl_ClearFlag(i);
			break;
		}
	}
}


void canfunc_RTR_SpeedAngle(CAN_HandleTypeDef *can, CAN_SpeedBLDC_AngleDC speedAngle)
{
	uint16_t deviceID = *(__IO uint32_t*)(0x08000000 + 64*1024);
	canfunc_MotorPutSpeedAndAngle(speedAngle);
	canctrl_Send(can, deviceID);
}

void canfunc_RTR_PID(CAN_HandleTypeDef *can, PID_Param pid, PID_type type)
{
	uint16_t deviceID = *(__IO uint32_t*)(0x08000000 + 64*1024);
	while(canfunc_PutAndSendParamPID(can, deviceID,  pid, type) != HAL_OK);
}

bool canfunc_GetBoolValue()
{
	uint8_t temp;
	if(canctrl_GetMessage(&temp, sizeof(uint8_t)) != HAL_OK) while(1);
	bool bVal = temp - 1;
	return bVal;
}

void canfunc_SetBoolValue(bool bVal, CAN_MODE_ID modeID)
{
	if(modeID != CANCTRL_MODE_TEST
	&& modeID != CANCTRL_MODE_PID_BLDC_BREAKPROTECTION
	&& modeID != CANCTRL_MODE_SET_HOME
	&& modeID != CANCTRL_MODE_MOTOR_BLDC_BRAKE
	&& modeID != CANCTRL_MODE_UNTANGLE_WIRE
	) return;
	canctrl_SetID(modeID);
	uint8_t temp = (uint8_t)bVal + 1;
	canctrl_PutMessage((void*)&temp, 1);
}

uint32_t canfunc_MotorGetEncoderPulseBLDC()
{
	uint32_t encBLDC = 0;
	if(canctrl_GetMessage(&encBLDC,sizeof(encBLDC))!= HAL_OK) while(1);
	return encBLDC;
}

void canfunc_MotorPutSpeedAndAngle(CAN_SpeedBLDC_AngleDC speedAngle)
{
	canctrl_SetID(CANCTRL_MODE_MOTOR_SPEED_ANGLE);
	canctrl_PutMessage((void*)&speedAngle, sizeof(CAN_SpeedBLDC_AngleDC));
}

CAN_SpeedBLDC_AngleDC canfunc_MotorGetSpeedAndAngle()
{
	CAN_SpeedBLDC_AngleDC speedAngle;
	if(canctrl_GetMessage(&speedAngle, sizeof(CAN_SpeedBLDC_AngleDC)) != HAL_OK) while(1);
	return speedAngle;
}


/**
 *
 * @param can CAN module available, in STM32F1 is hcan
 * @param targetID value of @arg CAN_DEVICE_ID enum
 * @param pid struct of PID_Param, can be extract from struct BoardParameter_t
 *
 */
HAL_StatusTypeDef canfunc_PutAndSendParamPID(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID, PID_Param pid, PID_type type)
{
	CAN_PID canPID;
	 switch (type){
	 case PID_BLDC_SPEED:
		 canctrl_SetID(CANCTRL_MODE_PID_BLDC_SPEED);
		 break;
	 case PID_DC_ANGLE:
		 canctrl_SetID(CANCTRL_MODE_PID_DC_ANGLE);
		 break;
	 case PID_DC_SPEED:
		 canctrl_SetID(CANCTRL_MODE_PID_DC_SPEED);
		 break;
	 default:
		 return HAL_ERROR;
		 break;
	 }
	//Need to send 5 parameters of PID: kP, kI, kD, alpha, deltaT
	canPID.kp = pid.kP;
	canPID.ki = pid.kI;
	canPID.kd = pid.kD;
	canPID.alpha = pid.alpha;
	canPID.deltaT = pid.deltaT;
	return canctrl_SendMultipleMessages(can, targetID, (void*)&canPID, sizeof(CAN_PID));
}

void canfunc_Convert_CAN_PID_to_PID_Param(CAN_PID canPID, PID_Param *pid)
{
	pid->kP = canPID.kp;
	pid->kI = canPID.ki;
	pid->kD = canPID.kd;
	pid->alpha = canPID.alpha;
	pid->deltaT = canPID.deltaT;
}

PID_type canfunc_GetTypePID(){
	CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
	uint32_t temp = (rxHeader.StdId & 0x0f)-6;
	return temp;
}
CAN_PID canPID1;
void canfunc_GetPID(void (*pCallback)(CAN_PID canPID,PID_type type))
{
	static CAN_PID canPID;
	if(!pCallback) return;
	if(canctrl_GetMultipleMessages((void*)&canPID1, sizeof(canPID)) == HAL_OK){
		pCallback(canPID1,canfunc_GetTypePID());
	}

}
