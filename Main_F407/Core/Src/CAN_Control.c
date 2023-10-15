/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: SpiritBoi
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
void canctrl_SetDLC(uint8_t DLC){txHeader.DLC = DLC;}
uint32_t canctrl_GetDLC(){return txHeader.DLC;}
uint32_t canctrl_GetID(){return txHeader.StdId;}
uint32_t canctrl_GetEvent(){return canEvent;}
void canctrl_SetTargetDevice(CAN_DEVICE_ID dev){ canctrl_SetID(dev << CAN_DEVICE_POS);}
CAN_RxHeaderTypeDef canctrl_GetRxHeader(){return rxHeader;}
void canctrl_RTR_SetToData(){txHeader.RTR = CAN_RTR_DATA;}
void canctrl_RTR_SetToRemote(){txHeader.RTR = CAN_RTR_REMOTE;}

#define TARGET_FLAG_GROUP canEvent
void canctrl_SetFlag(CAN_MODE_ID e){SETFLAG(TARGET_FLAG_GROUP,e);}
bool canctrl_CheckFlag(CAN_MODE_ID e){return CHECKFLAG(TARGET_FLAG_GROUP,e);}
void canctrl_ClearFlag(CAN_MODE_ID e){CLEARFLAG(TARGET_FLAG_GROUP,e);}


HAL_StatusTypeDef canctrl_SetID(uint32_t ID){
	if(ID > 0x7ff) return HAL_ERROR;
	txHeader.StdId |= ID;
	return HAL_OK;
}


HAL_StatusTypeDef canctrl_PutMessage(void* data,size_t dataSize)
{
	memset(txData,0,sizeof(txData));
	if(dataSize <= 8) txHeader.DLC = dataSize;
	memcpy(txData,data,sizeof(txData));
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_GetMessage(void *data, size_t sizeOfDataType){
	if(rxHeader.DLC != sizeOfDataType) return HAL_ERROR;
	memcpy(data,rxData,sizeOfDataType);
	return HAL_OK;
}


HAL_StatusTypeDef canctrl_SendMultipleMessages(CAN_HandleTypeDef *can,
											CAN_DEVICE_ID targetID,
											void *data,
											size_t sizeOfDataType)
{
	static bool IsBusy = false;
	static uint16_t tempTxDataLen = 0;
	if(IsBusy) return HAL_BUSY;
	if(!tempTxDataLen) {
		tempTxDataLen = sizeOfDataType;
	}
	if(tempTxDataLen >= 8 && tempTxDataLen <= sizeOfDataType) {
		txHeader.DLC = 8;
		memcpy(txData,data+(sizeOfDataType - tempTxDataLen),8);
		tempTxDataLen -= txHeader.DLC;
	}
	else if(tempTxDataLen < 8){
		txHeader.DLC = tempTxDataLen;
		memcpy(txData,data+(sizeOfDataType - tempTxDataLen),txHeader.DLC);
		tempTxDataLen = 0;
		if(canctrl_Send(can, targetID) == HAL_OK){
			memset(txData,0,sizeof(txData));
			IsBusy = false;
		}
		return HAL_OK;
	}
	IsBusy = true;
	if(canctrl_Send(can, targetID) == HAL_OK){
		memset(txData,0,sizeof(txData));
		IsBusy = false;
	}
	return HAL_BUSY;
}

HAL_StatusTypeDef canctrl_GetMultipleMessages(void *data, size_t sizeOfDataType)
{
	static uint16_t tempRxDataLen = 0;
	static uint32_t stdID_PreMesg = 0;

	uint32_t canMaskMode = rxHeader.StdId & 0x0f;
	//If this is a new message (which is tempRxDataLen = 0), set stdID_PreMesg to received ID of rxHeader
	if(((canMaskMode) != stdID_PreMesg) && !tempRxDataLen){
		stdID_PreMesg = canMaskMode;
		// copy first data to output data, increase length received data
		memcpy(data+tempRxDataLen,rxData,rxHeader.DLC);
		tempRxDataLen += rxHeader.DLC;
		canEvent = 0;
	}
	//If data already exist but the previous message ID is not match with current received ID, return ERROR
	else if(((canMaskMode) != stdID_PreMesg) && tempRxDataLen){
		canEvent = 0;
		return HAL_ERROR;
	}
	//If the received ID is match with the previous message
	else if((canMaskMode) == stdID_PreMesg){
		memcpy(data+tempRxDataLen,rxData,rxHeader.DLC);
		tempRxDataLen += rxHeader.DLC;
		canEvent = 0;
	}
	if(tempRxDataLen == sizeOfDataType){
		return HAL_OK;
	} else return HAL_BUSY;
}

HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID)
{
	if(!txHeader.DLC) return HAL_ERROR;
	if(!HAL_CAN_GetTxMailboxesFreeLevel(can)) return HAL_ERROR;
	HAL_StatusTypeDef err = HAL_OK;
	txHeader.IDE = CAN_ID_STD;
	canctrl_RTR_SetToData();
	if(targetID) canctrl_SetTargetDevice(targetID);
	err = HAL_CAN_AddTxMessage(can, &txHeader, txData, txMailBox);
	txHeader.StdId = 0;
	memset(txData,0,sizeof(txData));
	return err;
}

void checkEventFromRxHeader(){
	for(uint8_t i = CANCTRL_MODE_START + 1; i < CANCTRL_MODE_END;i++){
		// masking out CAN_DEVICE_ID, only mode and reverse bit remain
		if((rxHeader.StdId & 0x0f) ==  i)	{
			canctrl_SetFlag(i);
			break;
		}
	}
}


HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO)
{
	//Note: bắt buộc phải gọi hàm này để xử lý, nếu comment hàm này thì ngắt CAN sẽ liên tục gọi tới HAL_CAN_RxFifo0MsgPendingCallback vì
	// message đang chờ không xử lý và không cho phép chạy chương trình chính (liên tục nhảy vào chương trình ngắt)
	HAL_CAN_GetRxMessage(can, FIFO, &rxHeader, rxData);
	checkEventFromRxHeader();
	return HAL_OK;
}

void canctrl_GetRxData(uint8_t *outData)
{
	memcpy(outData,rxData,rxHeader.DLC);
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
	canFilCfg.FilterIdLow = 		ID1 << 5; // 0010000000100000
	canFilCfg.FilterIdHigh = 		ID2 << 5; // 0010000001000000
	canFilCfg.FilterMaskIdLow = 	ID3 << 5; // 0010000001100000
	canFilCfg.FilterMaskIdHigh = 	ID4 << 5; // 0010000010000000
	canFilCfg.FilterMode = CAN_FILTERMODE_IDLIST;
	canFilCfg.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilCfg.SlaveStartFilterBank = 13;
	return HAL_CAN_ConfigFilter(can, &canFilCfg);
}

// 0000 0000 0000 0000
// {  ID       }

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
