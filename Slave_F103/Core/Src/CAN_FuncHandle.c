/*
 * CAN_FuncHandle.c
 *
 *  Created on: Sep 22, 2023
 *      Author: KHOA
 */

#include "CAN_Control.h"
#include "CAN_FuncHandle.h"
#include <string.h>
#include <stdbool.h>
union fByte speedMotor,angleMotor;
union fByte pidDC_Speed,pidDC_Pos,pidBLDC_Speed;



void canfunc_HandleRxEvent(void(*pCallback)(CAN_MODE_ID ID))
{
	uint32_t ce = canctrl_GetEvent();
	if(!ce) return;
	for(uint8_t i = CANCTRL_MODE_START + 1; i < CANCTRL_MODE_END - 1; i++){
		if(canctrl_CheckFlag(i)) {
			pCallback(i);
			canctrl_ClearFlag(i);
			break;//only one event to handle for incoming message
		}

	}

}


void canfunc_MotorSetBrake(bool brake)
{
	canctrl_SetID(CANCTRL_MODE_MOTOR_BLDC_BRAKE);
	canctrl_PutMessage((void*)&brake, 1);
}

bool canfunc_MotorGetBrake()
{
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	return canctrl_GetIntNum();
}

HAL_StatusTypeDef canfunc_MotorPutEncoderPulse(uint16_t encBLDC, uint16_t encDC)
{
	canctrl_SetID(CANCTRL_MODE_ENCODER);
	uint32_t temp = 0;
	temp = encDC | encBLDC << 16;
	canctrl_PutMessage((void*)&temp, 4);
	return HAL_OK;
}

void canfunc_MotorGetEncoderPulse(int16_t *encBLDC, int16_t *encDC)
{
	if(!canctrl_CheckFlag(CANCTRL_MODE_ENCODER)) return;
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	memcpy(encBLDC,rxData,2);
	memcpy(encDC,rxData + 2,2);
//	convBigEndianToLittleEndian((uint8_t*)encBLDC, 2);
//	convBigEndianToLittleEndian((uint8_t*)encDC, 2);
	canctrl_CheckFlag(CANCTRL_MODE_ENCODER);
}


HAL_StatusTypeDef canfunc_MotorPutSpeedAndRotation(float speed, float angle)
{
	canctrl_SetID(CANCTRL_MODE_MOTOR_SPEED_ANGLE);
	speedMotor.floatData = speed;
	angleMotor.floatData = angle;
	uint8_t canData[8] = {0};
	memcpy(canData,speedMotor.byteData,sizeof(speedMotor.byteData));
	memcpy(canData + sizeof(float),angleMotor.byteData,sizeof(float));
	canctrl_PutMessage((void*)canData, sizeof(canData));
	return HAL_OK;
}

void canfunc_MotorGetSpeedAndRotation(float *speed, float *angle)
{

	if(!canctrl_CheckFlag(CANCTRL_MODE_MOTOR_SPEED_ANGLE)) return;
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	memcpy(speedMotor.byteData,rxData + sizeof(float),sizeof(float));
	memcpy(angleMotor.byteData,rxData,sizeof(float));
//	convBigEndianToLittleEndian(speedMotor.byteData, 4);
//	convBigEndianToLittleEndian(angleMotor.byteData, 4);
	*speed = speedMotor.floatData;
	*angle = angleMotor.floatData;
}


