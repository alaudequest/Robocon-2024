/*
 * CAN_Control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: KHOA
 */
#include "CAN_Control.h"

uint32_t TxMailBox[3];
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
union float_byte{
	float floatdata[2];
	uint8_t bytedata[8];
}float_byte;

HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t DLC, uint32_t RTR)
{
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.RTR = RTR;
	  TxHeader.StdId = ID;
	  TxHeader.DLC = DLC;
	  return HAL_OK;
}

HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can,uint8_t *txdata, uint16_t txDataSize)
{
	HAL_CAN_GetTxMailboxesFreeLevel(can);
	return HAL_CAN_AddTxMessage(can, &TxHeader, txdata, TxMailBox);
}

HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can,uint8_t *rxdata, uint32_t FIFO)
{
	if(FIFO != CAN_RX_FIFO0 || FIFO != CAN_RX_FIFO1) return HAL_ERROR;
	return HAL_CAN_GetRxMessage(can, FIFO, &RxHeader, rxdata);
}

HAL_StatusTypeDef canctrl_SetFilter(CAN_HandleTypeDef *can, uint32_t FIFO, uint32_t ID, uint8_t filterbank){
	CAN_FilterTypeDef canfilterconfig;

	if(FIFO != CAN_FILTER_FIFO0 || FIFO != CAN_FILTER_FIFO1) return HAL_ERROR;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = filterbank;
	canfilterconfig.FilterFIFOAssignment = FIFO;
	canfilterconfig.FilterIdHigh = ID << 5;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = ID << 5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 13;

	HAL_CAN_ConfigFilter(can, &canfilterconfig);
	return HAL_OK;
}

HAL_StatusTypeDef canctrl_Brake(CAN_HandleTypeDef *can){
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x50 << 5;
	canfilterconfig.FilterMaskIdHigh = 0x80 << 5;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 13;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
}

HAL_StatusTypeDef canctrl_Init(CAN_HandleTypeDef *can){
	canctrl_Brake(can);
}
