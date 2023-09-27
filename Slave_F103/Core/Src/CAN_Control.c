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
uint32_t canEvent;
iByte ibyte;
fByte fbyte;
void canctrl_SetDLC(uint8_t DLC){txHeader.DLC = DLC;}
uint32_t canctrl_GetDLC(){return txHeader.DLC;}
uint32_t canctrl_GetID(){return txHeader.StdId;}
uint32_t canctrl_GetEvent(){return canEvent;}
void canctrl_SetTargetDevice(CAN_DEVICE_ID dev){ canctrl_SetID(dev << CAN_DEVICE_POS);}
CAN_RxHeaderTypeDef canctrl_GetRxHeader(){return rxHeader;}
void canctrl_RTR_SetToData(){txHeader.RTR = CAN_RTR_DATA;}
void canctrl_RTR_SetToRemote(){txHeader.RTR = CAN_RTR_REMOTE;}

void canctrl_SetFlag(CAN_MODE_ID flag){CAN_EVT_SETFLAG(flag);}
void canctrl_ClearFlag(CAN_MODE_ID flag){CAN_EVT_CLEARFLAG(flag);}
bool canctrl_CheckFlag(CAN_MODE_ID flag){return CAN_EVT_CHECKFLAG(flag);}
HAL_StatusTypeDef canctrl_SetID(uint32_t ID){
	if(ID > 0x7ff) return HAL_ERROR;
	txHeader.StdId |= ID;
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_PutMessage(void* data,size_t dataSize)
{

	memset(txData,0,sizeof(txData));
	txHeader.DLC = 0;
	uint8_t *temp = (uint8_t*)data;
	for(int8_t i = 0; i < dataSize;i++){
		if(*(temp+i)){
			if(!txHeader.DLC) txHeader.DLC = dataSize - i;
			txData[i] = *(temp+i);
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID)
{
	if(!txHeader.DLC) return HAL_ERROR;
	if(!HAL_CAN_GetTxMailboxesFreeLevel(can)) return HAL_ERROR;
	txHeader.IDE = CAN_ID_STD;
	canctrl_RTR_SetToData();
	if(targetID) canctrl_SetTargetDevice(targetID);
	HAL_CAN_AddTxMessage(can, &txHeader, txData, txMailBox);
	txHeader.StdId = 0;
	memset(txData,0,sizeof(txData));
	return HAL_OK;
}

void checkEventFromRxHeader(){
	for(uint8_t i = CANCTRL_MODE_START + 1; i < CANCTRL_MODE_END;i++){
		if((rxHeader.StdId & 0xff) ==  i)	{
			CAN_EVT_SETFLAG(i);
			break;
		}
	}
}
HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO)
{
	HAL_CAN_GetRxMessage(can, FIFO, &rxHeader, rxData);
	checkEventFromRxHeader();
	return HAL_OK;
}

void canctrl_GetRxData(uint8_t *outData)
{
	memcpy(outData,rxData,rxHeader.DLC);
}




uint64_t canctrl_GetIntNum()
{
	canctrl_GetRxData(ibyte.byteData);
	memset(rxData,0,sizeof(rxData));
	rxHeader.DLC = 0;
	return ibyte.intData;
}

float canctrl_GetFloatNum()
{
	if(rxHeader.DLC > 4) return 0;
	canctrl_GetRxData(fbyte.byteData);
	memset(rxData,0,sizeof(rxData));
	rxHeader.DLC = 0;
	return fbyte.floatData;
}



HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t RTR)
{
	  txHeader.IDE = CAN_ID_STD;
	  if(RTR == CAN_RTR_DATA) canctrl_RTR_SetToData();
	  else canctrl_RTR_SetToRemote();
	  canctrl_SetID(ID);
	  return HAL_OK;
}

HAL_StatusTypeDef canctrl_Filter_List16(CAN_HandleTypeDef *can,
												uint16_t ID1,
												uint16_t ID2,
												uint16_t ID3,
												uint16_t ID4,
												uint32_t filBank,
												uint32_t FIFO)
{
	CAN_FilterTypeDef canFilCfg;
	canFilCfg.FilterActivation = CAN_FILTER_ENABLE;
	canFilCfg.FilterBank = filBank;
	canFilCfg.FilterFIFOAssignment = FIFO;
	canFilCfg.FilterIdLow = 		ID1 << 5;
	canFilCfg.FilterIdHigh = 		ID2 << 5;
	canFilCfg.FilterMaskIdLow = 	ID3 << 5;
	canFilCfg.FilterMaskIdHigh = 	ID4 << 5;
	canFilCfg.FilterMode = CAN_FILTERMODE_IDLIST;
	canFilCfg.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilCfg.SlaveStartFilterBank = 13;
	return HAL_CAN_ConfigFilter(can, &canFilCfg);
}

HAL_StatusTypeDef canctrl_Filter_Mask16(CAN_HandleTypeDef *can,
										uint16_t highID,
										uint16_t lowID,
										uint16_t maskHigh,
										uint16_t maskLow,
										uint32_t filBank,
										uint32_t FIFO)
{
	CAN_FilterTypeDef canFilCfg;
	canFilCfg.FilterActivation = CAN_FILTER_ENABLE;
	canFilCfg.FilterBank = filBank;
	canFilCfg.FilterFIFOAssignment = FIFO;
	canFilCfg.FilterIdLow = 		lowID;
	canFilCfg.FilterIdHigh = 		highID;
	canFilCfg.FilterMaskIdLow = 	maskLow;
	canFilCfg.FilterMaskIdHigh = 	maskHigh;
	canFilCfg.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilCfg.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilCfg.SlaveStartFilterBank = 13;
	return HAL_CAN_ConfigFilter(can, &canFilCfg);
}


