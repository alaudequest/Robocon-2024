/*
 * CAN_FuncHandle.h
 *
 *  Created on: Sep 22, 2023
 *      Author: SpiritBoi
 */

#ifndef INC_CAN_FUNCHANDLE_H_
#define INC_CAN_FUNCHANDLE_H_
#include "main.h"
#include "BoardParameter.h"

typedef struct CAN_SpeedBLDC_AngleDC{
	float bldcSpeed;
	float dcAngle;
}CAN_SpeedBLDC_AngleDC;

typedef struct CAN_PID{
	float kp;
	float ki;
	float kd;
	float alpha;
	float deltaT;
}CAN_PID;

void canfunc_HandleRxEvent(void(*pCallback)(CAN_MODE_ID ID));

bool canfunc_GetBoolValue();
void canfunc_SetBoolValue(bool bVal, CAN_MODE_ID modeID);

uint32_t canfunc_MotorGetEncoderPulseBLDC();
void canfunc_MotorPutEncoderPulseBLDC(uint32_t encBLDC);

void canfunc_MotorPutSpeedAndAngle(CAN_SpeedBLDC_AngleDC speedAngle);
CAN_SpeedBLDC_AngleDC canfunc_MotorGetSpeedAndAngle();

void canfunc_Convert_CAN_PID_to_PID_Param(CAN_PID canPID, PID_Param *pid);
void canfunc_GetPID(void (*pCallback)(CAN_PID canPID,PID_type type));
HAL_StatusTypeDef canfunc_PutAndSendParamPID(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID, PID_Param pid, PID_type type);

void canfunc_RTR_SpeedAngle(CAN_HandleTypeDef *can, CAN_SpeedBLDC_AngleDC speedAngle);
void canfunc_RTR_PID(CAN_HandleTypeDef *can, PID_Param pid, PID_type type);


#endif /* INC_CAN_FUNCHANDLE_H_ */
