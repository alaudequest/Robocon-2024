/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: KHOA
 */
#include "CAN_Control.h"
#include <stdio.h>
#include <string.h>


union fByte{
	float floatData;
	uint8_t byteData[4];
}fByte;


union iByte{
	uint64_t intData;
	uint8_t byteData[8];
}iByte;

uint32_t txMailBox[3];
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint8_t txData[8] = {0};
uint8_t rxData[8] = {0};
union fByte speedMotor,angleMotor;
uint32_t canEvent;

void canctrl_SetDLC(uint8_t DLC){txHeader.DLC = DLC;}
uint32_t canctrl_GetDLC(){return txHeader.DLC;}

HAL_StatusTypeDef canctrl_SetID(uint32_t ID){
	if(ID > 0x7ff) return HAL_ERROR;
	txHeader.StdId = ID;
	return HAL_OK;
}
uint32_t canctrl_GetID(){return txHeader.StdId;}
CAN_RxHeaderTypeDef canctrl_GetRxHeader(){return rxHeader;}
void canctrl_RTR_SetToData(){txHeader.RTR = CAN_RTR_DATA;}
void canctrl_RTR_SetToRemote(){
	txHeader.RTR = CAN_RTR_REMOTE;
//	txHeader.DLC = 0;
}



void canctrl_SetFlag(CAN_EVT flag){CAN_EVT_SETFLAG(flag);}
void canctrl_ClearFlag(CAN_EVT flag){CAN_EVT_CLEARFLAG(flag);}
bool canctrl_CheckFlag(CAN_EVT flag){return CAN_EVT_CHECKFLAG(flag);}


HAL_StatusTypeDef canctrl_PutMessage(void* data,size_t dataSize)
{
	memset(txData,0,sizeof(txData));
	txHeader.DLC = 0;
	uint8_t *temp = (uint8_t*)data;
	for(int8_t i = dataSize - 1; i > -1 ;i--){
		if(*(temp+i)){
			if(!txHeader.DLC) txHeader.DLC = i;
			txData[txHeader.DLC - i] = *(temp+i);
		}
	}
	txHeader.DLC ++;
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can, uint32_t ID)
{
	if(!txHeader.DLC) return HAL_ERROR;
	txHeader.IDE = CAN_ID_STD;
	canctrl_RTR_SetToData();
//	canctrl_SetID(ID);
	HAL_CAN_AddTxMessage(can, &txHeader, txData, txMailBox);
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO)
{
//	if(FIFO != CAN_RX_FIFO0 || FIFO != CAN_RX_FIFO1) return HAL_ERROR;

	HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
	HAL_CAN_GetRxMessage(can, FIFO, &rxHeader, rxData);
	if(FIFO == CAN_RX_FIFO0) CAN_EVT_SETFLAG(CAN_EVT_RX_FIFO0);
	else if(FIFO == CAN_RX_FIFO1) CAN_EVT_SETFLAG(CAN_EVT_RX_FIFO1);
	else return HAL_ERROR;


	if(rxHeader.StdId & CAN_ID_BRAKE_MASK) CAN_EVT_SETFLAG(CAN_EVT_BRAKE_MOTOR);
	else if(rxHeader.StdId & CAN_ID_ENCODER_MASK) CAN_EVT_SETFLAG(CAN_EVT_GET_ENCODER);
	else if(rxHeader.StdId & CAN_ID_SPEED_ANGLE_MASK) CAN_EVT_SETFLAG(CAN_EVT_SPEED_ANGLE);
	else if(rxHeader.StdId & CAN_ID_SET_HOME) CAN_EVT_SETFLAG(CAN_EVT_SET_HOME);
	return HAL_OK;
}

void canctrl_GetRxData(uint8_t *outData)
{
	memcpy(outData,rxData,rxHeader.DLC);
}

void convBigEndianToLittleEndian(uint8_t *data, size_t length){
	if (length < 2 || length > 8) return;

	uint8_t *bytes = (uint8_t *)data;

	// Swap the bytes to convert from Big Endian to Little Endian
	for (size_t i = 0; i < length / 2; i++) {
		uint8_t temp = bytes[i];
		bytes[i] = bytes[length - 1 - i];
		bytes[length - 1 - i] = temp;
	}
}


uint64_t canctrl_GetIntNum()
{
	canctrl_GetRxData(iByte.byteData);
	convBigEndianToLittleEndian(iByte.byteData,rxHeader.DLC);
	memset(rxData,0,sizeof(rxData));
	rxHeader.DLC = 0;
	return iByte.intData;
}

float canctrl_GetFloatNum()
{
	if(rxHeader.DLC > 4) return 0;
	canctrl_GetRxData(fByte.byteData);
	convBigEndianToLittleEndian(fByte.byteData,rxHeader.DLC);
	memset(rxData,0,sizeof(rxData));
	rxHeader.DLC = 0;
	return fByte.floatData;
}

void canctrl_MotorSendBrakeMessage(CAN_HandleTypeDef *can, CAN_ID motorCtrlID, bool brake)
{
	canctrl_SetID((motorCtrlID << CANCTRL_ID_DEVICE_POS) | CAN_ID_BRAKE_MASK);
	canctrl_PutMessage((void*)&brake, 1);
	canctrl_Send(can, canctrl_GetID());
}

HAL_StatusTypeDef canctrl_MotorPutEncoderPulse(CAN_ID motorCtrlID, int16_t encBLDC, int16_t encDC)
{
	if(motorCtrlID > 4 || !motorCtrlID) return HAL_ERROR;
	canctrl_SetID((motorCtrlID << CANCTRL_ID_DEVICE_POS) | CAN_ID_ENCODER_MASK);
	uint32_t temp = encBLDC << 16 | encDC;
	canctrl_PutMessage((void*)&temp, 4);
	return HAL_OK;
}

void canctrl_MotorGetEncoderPulse(int16_t *encBLDC, int16_t *encDC)
{
	if(!CAN_EVT_CHECKFLAG(CAN_EVT_GET_ENCODER)) return;
	memcpy(encBLDC,rxData,2);
	memcpy(encDC,rxData + 2,2);
	convBigEndianToLittleEndian((uint8_t*)encBLDC, 2);
	convBigEndianToLittleEndian((uint8_t*)encDC, 2);
	CAN_EVT_CLEARFLAG(CAN_EVT_GET_ENCODER);
}


HAL_StatusTypeDef canctrl_MotorSetSpeedAndRotation(CAN_ID motorCtrlID, float speed, float angle)
{
	if(angle > 360 || motorCtrlID > 4) return HAL_ERROR;
	canctrl_SetID(motorCtrlID << CANCTRL_ID_DEVICE_POS | CAN_ID_SPEED_ANGLE_MASK);
	speedMotor.floatData = speed;
	angleMotor.floatData = angle;
	uint8_t canData[8] = {0};
	memcpy(canData,speedMotor.byteData,sizeof(speedMotor.byteData));
	memcpy(canData + sizeof(float),angleMotor.byteData,sizeof(float));
	canctrl_PutMessage((void*)canData, sizeof(canData));

	return HAL_OK;
}

void canctrl_MotorGetSpeedAndRotation(float *speed, float *angle)
{
	if(!CAN_EVT_CHECKFLAG(CAN_EVT_SPEED_ANGLE)) return;
	memcpy(speedMotor.byteData,rxData + sizeof(float),sizeof(float));
	memcpy(angleMotor.byteData,rxData,sizeof(float));
	convBigEndianToLittleEndian(speedMotor.byteData, 4);
	convBigEndianToLittleEndian(angleMotor.byteData, 4);
	*speed = speedMotor.floatData;
	*angle = angleMotor.floatData;
}


HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t RTR)
{
	  txHeader.IDE = CAN_ID_STD;
	  if(RTR == CAN_RTR_DATA) canctrl_RTR_SetToData();
	  else canctrl_RTR_SetToRemote();
	  canctrl_SetID(ID);
	  return HAL_OK;
}

HAL_StatusTypeDef canctrl_FilCfg(CAN_HandleTypeDef *can, uint32_t filterID, uint32_t filBank, uint32_t FIFO){
	CAN_FilterTypeDef canFilCfg;
	canFilCfg.FilterActivation = CAN_FILTER_ENABLE;
	canFilCfg.FilterBank = filBank;
	canFilCfg.FilterFIFOAssignment = FIFO;
	canFilCfg.FilterIdHigh = filterID << 5;
	canFilCfg.FilterMaskIdHigh = filterID << 5;
	canFilCfg.FilterIdLow = 0x0000;
	canFilCfg.FilterMaskIdLow = 0x0000;
	canFilCfg.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilCfg.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilCfg.SlaveStartFilterBank = 13;
	return HAL_CAN_ConfigFilter(can, &canFilCfg);
}


