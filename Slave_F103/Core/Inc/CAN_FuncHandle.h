/*
 * CAN_FuncHandle.h
 *
 *  Created on: Sep 22, 2023
 *      Author: KHOA
 */

#ifndef INC_CAN_FUNCHANDLE_H_
#define INC_CAN_FUNCHANDLE_H_
#include "main.h"
#include "PID.h"
#include "BoardParameter.h"
void canfunc_HandleRxEvent(void(*pCallback)(CAN_MODE_ID ID));

uint8_t canfunc_MotorGetBrake();
void canfunc_MotorSetBrake(bool brake);


uint32_t canfunc_MotorGetEncoderPulseBLDC();
void canfunc_MotorPutEncoderPulseBLDC(uint32_t encBLDC);

void canfunc_MotorGetSpeedAndAngle(float *speed, float *angle);
HAL_StatusTypeDef canfunc_MotorPutSpeedAndAngle(float speed, float angle);

void canfunc_GetPID();
void canfunc_PutAndSendParamPID(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID, PID_Param pid, PID_type type);

uint8_t canfunc_GetTestMode();
void canfunc_SetTestMode(bool IsTestMode);

void canfunc_EnableSendPID();
bool canfunc_GetStateEnableSendPID();

#endif /* INC_CAN_FUNCHANDLE_H_ */
