/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: KHOA
 */
#include "CAN_Control.h"
#include <stdio.h>
#include <string.h>
uint32_t txMailBox[3];
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint8_t txData[8] = {0};
uint8_t rxData[8] = {0};
union fByte{
	float floatData;
	uint8_t byteData[4];
}fByte;

union iByte{
	uint64_t intData;
	uint8_t byteData[8];
}iByte;
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

HAL_StatusTypeDef canctrl_PutMessage(uint64_t data)
{
	memset(txData,0,sizeof(txData));
	txHeader.DLC = 0;
	uint8_t temp;
	for(int8_t i = sizeof(txData) - 1; i > -1 ;i--){
		temp = (data >> i*8) & 0xff;
		if(temp){
			if(!txHeader.DLC) txHeader.DLC = i;
			txData[txHeader.DLC - i] = temp;
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
	canctrl_SetID(ID);
	HAL_CAN_AddTxMessage(can, &txHeader, txData, txMailBox);
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO)
{
//	if(FIFO != CAN_RX_FIFO0 || FIFO != CAN_RX_FIFO1) return HAL_ERROR;
	return HAL_CAN_GetRxMessage(can, FIFO, &rxHeader, rxData);
}

void canctrl_GetRxData(uint8_t *outData){
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
	canctrl_GetRxData(fByte.byteData);
	convBigEndianToLittleEndian(fByte.byteData,rxHeader.DLC);
	memset(rxData,0,sizeof(rxData));
	rxHeader.DLC = 0;
	return fByte.floatData;
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


