/*
 * AppInterface.c
 *
 *  Created on: Jan 24, 2024
 *      Author: SpiritBoi
 */
#include "AppInterface.h"


char uartRxData[APP_BUFFER_SIZE] = {0};
char uartTxData[APP_BUFFER_SIZE] = {0};
UART_HandleTypeDef *pAppUART;
bool isRxBufferEmpty = true;
bool isTxBufferEmpty = true;
FrameData fd;
void *pArg[CMD_End - 1] = {0};
uint8_t sizeArgument[CMD_End - 1] = {0};

pCpltCallback pCallback;
pErrorCallback pErr;
static void ResetFrameData();
static bool IsPassCRC();
AppErrorCode ProcessCommandList();
void appintf_Init(UART_HandleTypeDef *huart, BoardID boardID) {
	HAL_UART_Receive_IT(huart, (uint8_t*) uartRxData, APP_DATA_LENGTH);
	pAppUART = huart;
}

void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist) {
	if(cmdlist == CMD_Start || cmdlist == CMD_End) return;
	pArg[cmdlist] = arg;
	sizeArgument[cmdlist] = sizeOfArgument;
}

void appintf_RegisterReceivedCallbackEvent(void (*pCpltCallback)(CommandList cmdlist)) {
	pCallback = pCpltCallback;
}

void appintf_RegisterErrorCallbackEvent(void (*pErrorCallback)(AppErrorCode err)) {
	pErr = pErrorCallback;
}

static void GetValueFromPayload() {
	if(fd.payloadLength != sizeArgument[fd.cmdList])
		if(pErr != NULL)
			pErr(APPERR_PAYLOAD_NOT_RECOGNIZE);
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	if(pArg[fd.cmdList] != NULL)
		memcpy(pArg[fd.cmdList], uartRxData + payloadField, fd.payloadLength); //offset to payloadField in uartRxData
}


static void DecodeFrameData() {
	uint32_t crcField = APP_DATA_LENGTH + APP_COMMAND_LIST_LENGTH + fd.payloadLength;
	uint32_t crcNibbleByteMSB = uartRxData[crcField] << 8;
	uint32_t crcNibbleByteLSB = uartRxData[crcField + 1];
	fd.cmdList = uartRxData[1];
	fd.crc16 = crcNibbleByteMSB | crcNibbleByteLSB;
	if(!IsPassCRC())
		if(pErr != NULL)
			pErr(APPERR_CRC_FAIL);

}

/**
 * @brief Handle frame data received, must be placed inside <void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)> function
 * @example
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 * 		appintf_HandleReceive(huart);
 * }
 * @param huart
 */
void appintf_HandleReceive(UART_HandleTypeDef *huart) {
	if(huart != pAppUART)
		if(pErr != NULL)
			pErr(APPERR_UART_PORT_NULL);
	static bool isOnFrameReceived = false;
	if(fd.isOnProcess) return;
	if(!isOnFrameReceived) {
		fd.isOnProcess = true;
		fd.payloadLength = uartRxData[0];
		uint32_t totalLength = fd.totalLength = fd.payloadLength + APP_CRC_LENGTH + APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
		if(totalLength < sizeof(uartRxData))
			HAL_UART_Receive_IT(pAppUART, (uint8_t*) uartRxData + APP_DATA_LENGTH, totalLength - APP_DATA_LENGTH);
		else
		if(pErr != NULL) pErr(APPERR_OUT_OF_BUFFER_SIZE);
		isOnFrameReceived = true;
		fd.isOnProcess = false;
	}
	else {
		fd.isOnProcess = true;
		isOnFrameReceived = false;
		DecodeFrameData();
		GetValueFromPayload();
		HAL_UART_Receive_IT(pAppUART, (uint8_t*) uartRxData, APP_DATA_LENGTH);
		memset(uartRxData, 0, sizeof(uartRxData));
		fd.isOnProcess = false;
		ResetFrameData();
		if(pCallback != NULL) pCallback(fd.cmdList);
	}

}



AppErrorCode appintf_MakeFrame(CommandList cmdlist) {
	uint32_t totalLength = fd.totalLength = APP_DATA_LENGTH
			+ APP_COMMAND_LIST_LENGTH
			+ sizeArgument[cmdlist]
			+ APP_CRC_LENGTH;
	if(totalLength > sizeof(uartTxData))
		return APPERR_OUT_OF_BUFFER_SIZE;
	if(!pArg[cmdlist]) return APPERR_STORE_BUFFER_IS_NULL;
	if(!isTxBufferEmpty) memset(uartTxData, 0, sizeof(uartTxData));
	fd.isOnProcess = true;
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint16_t crcResult;
	uint32_t crcField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH + sizeArgument[cmdlist];
	fd.payloadLength = sizeArgument[cmdlist];
	crcResult = crc16_Unreflected((uint8_t*) pArg[cmdlist], sizeArgument[cmdlist], 0);
	uint8_t temp[2] = {crcResult >> 8, crcResult & 0xff};
	uartTxData[0] = sizeArgument[cmdlist];
	uartTxData[1] = (uint8_t) cmdlist;
	memcpy(uartTxData + payloadField, pArg[cmdlist], sizeArgument[cmdlist]);
	memcpy(uartTxData + crcField, temp, 2);
	isTxBufferEmpty = false;
	return APPERR_OK;
}

void appintf_SendFrame() {
	if(!fd.isOnProcess) return;
	HAL_UART_Transmit(pAppUART, (uint8_t*) uartTxData, fd.totalLength, 10);
	memset(uartTxData, 0, sizeof(uartTxData));
	isTxBufferEmpty = true;
	ResetFrameData();
}

static void ResetFrameData() {
	FrameData fdTemp = {0};
	fd = fdTemp;
}

static bool IsPassCRC() {
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint8_t *pPayloadCRC = (uint8_t*) uartRxData + payloadField;
	if(crc16_Unreflected(pPayloadCRC, fd.payloadLength + APP_CRC_LENGTH, 0)) {
		return false;
	}
	return true;
}

