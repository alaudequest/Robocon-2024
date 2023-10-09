/*
 * CAN_FuncHandle.c
 *
 *  Created on: Sep 22, 2023
 *      Author: KHOA
 */

#include "CAN_Control.h"
#include "CAN_FuncHandle.h"
#include "PID.h"
#include <string.h>
#include <stdbool.h>

union fByte speedMotor,angleMotor;
bool pidSendEnable = 0;

void canfunc_HandleRxEvent(void(*pCallback)(CAN_MODE_ID ID))
{
	uint32_t ce = canctrl_GetEvent();
	if(!ce) return;
	for(uint8_t i = CANCTRL_MODE_START + 1; i < CANCTRL_MODE_END; i++){
		if(canctrl_CheckFlag(i)) {
			pCallback(i);
		}
	}
}

uint8_t canfunc_GetTestMode()
{
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	canctrl_ClearFlag(CANCTRL_MODE_TEST);
	uint8_t testMode = canctrl_GetIntNum();
	return testMode--;
}

void canfunc_SetTestMode(uint8_t IsTestMode)
{
	canctrl_SetID(CANCTRL_MODE_TEST);
	IsTestMode++;
	canctrl_PutMessage((void*)&IsTestMode, 1);
}


void canfunc_MotorSetBrake(uint8_t brake)
{
	canctrl_SetID(CANCTRL_MODE_MOTOR_BLDC_BRAKE);
	brake++;
	canctrl_PutMessage((void*)&brake, 1);
}

uint8_t canfunc_MotorGetBrake()
{
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	canctrl_ClearFlag(CANCTRL_MODE_MOTOR_BLDC_BRAKE);
	uint8_t brake = canctrl_GetIntNum();
	return brake--;
}

void canfunc_MotorSetBreakProtectionBLDC(uint8_t Break)
{
	canctrl_SetID(CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
	Break++;
	canctrl_PutMessage((void*)&Break, 1);
}

uint8_t  canfunc_MotorGetBreakProtectionBLDC()
{
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	canctrl_ClearFlag(CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
	uint8_t Break = canctrl_GetIntNum();
	return Break--;
}
void canfunc_MotorPutEncoderPulseBLDC(uint32_t encBLDC)
{
	canctrl_SetID(CANCTRL_MODE_ENCODER);
	canctrl_PutMessage((void*)&encBLDC, sizeof(encBLDC));
}

uint32_t canfunc_MotorGetEncoderPulseBLDC()
{
	uint32_t encBLDC = canctrl_GetIntNum();
	canctrl_ClearFlag(CANCTRL_MODE_ENCODER);
	return encBLDC;
}


HAL_StatusTypeDef canfunc_MotorPutSpeedAndAngle(float speed, float angle)
{
	canctrl_SetID(CANCTRL_MODE_MOTOR_SPEED_ANGLE);
	speedMotor.floatData = speed;
	angleMotor.floatData = angle;
	uint8_t canData[8] = {0};
	memcpy(canData,speedMotor.byteData,sizeof(float));
	memcpy(canData + sizeof(float),angleMotor.byteData,sizeof(float));
	canctrl_PutMessage((void*)canData, sizeof(canData));
	return HAL_OK;
}

void canfunc_MotorGetSpeedAndAngle(float *speed, float *angle)
{
	if(!canctrl_CheckFlag(CANCTRL_MODE_MOTOR_SPEED_ANGLE)) return;
	uint8_t rxData[8] = {0};
	canctrl_GetRxData(rxData);
	memcpy(speedMotor.byteData,rxData,sizeof(float));
	memcpy(angleMotor.byteData,rxData + sizeof(float),sizeof(float));
	*speed = speedMotor.floatData;
	*angle = angleMotor.floatData;
	canctrl_ClearFlag(CANCTRL_MODE_MOTOR_SPEED_ANGLE);
}




/**
 *
 * @param can CAN module available, in STM32F1 is hcan
 * @param targetID value of @arg CAN_DEVICE_ID enum
 * @param pid struct of PID_Param, can be extract from struct BoardParameter_t
 *
 */
void canfunc_PutAndSendParamPID(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID, PID_Param pid, PID_type type)
{
	if(!can || !pidSendEnable) return;
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
		 return;
		 break;
	 }
	//Need to send 5 parameters of PID: kP, kI, kD, alpha, deltaT
	uint8_t canData[8] = {0};
	static uint8_t i = 0;
	fByte a,b;
	switch(i){
	case 0:
		// put kP and kI to txData first
		a.floatData = pid.kP;
		b.floatData = pid.kI;
		break;
	case 1:
		// put kD and alpha to txData
		a.floatData = pid.kD;
		b.floatData = pid.alpha;
		break;
	case 2:
		// put the last one is deltaT to txData
		a.floatData = pid.deltaT;
		b.floatData = 0;
		break;
	}

	memcpy(canData,a.byteData,sizeof(float));
	memcpy(canData + sizeof(float),b.byteData,sizeof(float));
	canctrl_PutMessage((void*)canData, sizeof(canData));
	if(canctrl_Send(can, targetID) == HAL_OK) {
		if(i > 2) {
			i = 0;
			pidSendEnable = 0;
		}
		else i++;
		HAL_Delay(1000);
	}
}

PID_type PIDGetTypeFromEvent(uint32_t canEvent){
	if(canctrl_CheckFlag(CANCTRL_MODE_PID_DC_SPEED)) return PID_DC_SPEED;
	else if(canctrl_CheckFlag(CANCTRL_MODE_PID_DC_ANGLE)) return PID_DC_ANGLE;
	else if(canctrl_CheckFlag(CANCTRL_MODE_PID_BLDC_SPEED)) return PID_BLDC_SPEED;
	return 0;
}

/**
 * @test
 * Run debug mode, open Live Expression and type canEvent
 * type value of canEvent:
 * 0b0000100000000000 (CANCTRL_MODE_PID_BLDC_SPEED)
 * 0b0000010000000000 (CANCTRL_MODE_PID_DC_ANGLE)
 * 0b0000001000000000 (CANCTRL_MODE_PID_DC_SPEED)
 * change value of i from 0 to 2, bigger than 2
 */
void canfunc_GetPID()
{
	uint32_t canEvent = canctrl_GetEvent();
	static uint32_t canEventPre = 0;
	static uint8_t i = 0;
	fByte a,b;
	// if there is a new event get PID parameter, save it to event previous
	// by using bit mask
	if(!canEventPre) {
		canEventPre = 	(canctrl_CheckFlag(CANCTRL_MODE_PID_DC_SPEED))   ? (1 << CANCTRL_MODE_PID_DC_SPEED) :
						(canctrl_CheckFlag(CANCTRL_MODE_PID_DC_ANGLE))   ? (1 << CANCTRL_MODE_PID_DC_ANGLE) :
						(canctrl_CheckFlag(CANCTRL_MODE_PID_BLDC_SPEED)) ? (1 << CANCTRL_MODE_PID_BLDC_SPEED) : 0;
	}
	// if current event of Get PID is not the same with previous, reset all
	else if(!(canEventPre & canEvent)) {
		canEventPre = 0;
		i = 0;
	} else {
		PID_Param pid = brd_GetPID(PIDGetTypeFromEvent(canEvent));
		uint8_t rxData[8] = {0};
		canctrl_GetRxData(rxData);
		switch(i){
		case 1:
			// get kP and kI from rxData first
			memcpy(a.byteData,rxData,sizeof(float));
			memcpy(b.byteData,rxData + sizeof(float),sizeof(float));
			pid.kP = a.floatData;
			pid.kI = b.floatData;
			break;
		case 2:
			// get kD and alpha from rxData
			memcpy(a.byteData,rxData,sizeof(float));
			memcpy(b.byteData,rxData + sizeof(float),sizeof(float));
			pid.kD = a.floatData;
			pid.alpha = b.floatData;
			break;
		case 3:
			// get the last one is deltaT from rxData
			memcpy(a.byteData,rxData,sizeof(float));
			pid.deltaT = a.floatData;
			break;
		}
		brd_SetPID(pid,PIDGetTypeFromEvent(canEvent));
		uint32_t temp = canEventPre;
		for(uint8_t j = 1; j < 0xff; j++){
			temp /= 2;
			if(temp == 1) {
				canctrl_ClearFlag(j);
				break;
			}
		}
	}
	// reset state
	if(i > 3){
		canEventPre = 0;
		i = 0;
	} else i++;
}

void canfunc_EnableSendPID(){pidSendEnable = 1;}
bool canfunc_GetStateEnableSendPID(){return pidSendEnable;}


/**
 *
#include <stdio.h>
#include <stdint.h>
#include <string.h>
uint8_t txData[8] = {0};
uint8_t rxData[8] = {0};
typedef struct SpeedBLDC_AngleDC{
    float bldcSpeed;
    float dcAngle;
}SpeedBLDC_AngleDC;
void canctrl_PutMessage(void *data, size_t dataSize){
    if(dataSize > 8) return;
    memcpy(txData,data,sizeof(dataSize));
}
void canctrl_Send(){
    memcpy(rxData,txData,8);
}
void canctrl_GetMessage(void *data, size_t dataSize){
    memcpy(data,rxData,sizeof(dataSize));
}
int main()
{
    SpeedBLDC_AngleDC a,b;
    a.bldcSpeed = 10.54;
    a.dcAngle = -20.33;
    canctrl_PutMessage((void*)&a,sizeof(SpeedBLDC_AngleDC));
    canctrl_Send();
    canctrl_GetMessage((void*)&b,sizeof(SpeedBLDC_AngleDC));
    printf("b.bldcSpeed:%.2f\nb.dcAngle:%.2f",b.bldcSpeed,b.dcAngle);
    return 0;
}
 *
 *
 *
 *
 *
 */
