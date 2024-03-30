/*
 * AppInterface.c
 *
 *  Created on: Jan 24, 2024
 *      Author: SpiritBoi
 */
#include "AppInterface.h"

UART_HandleTypeDef *pAppUART;
bool isRxBufferEmpty = true;
bool isTxBufferEmpty = true;
FrameData fd;

pCpltCallback pCallback;
pErrorCallback pAppErr;
uint8_t *_pTxBuffer;
uint8_t *_pRxBuffer;
uint8_t _rxBufSize;
uint8_t _txBufSize;
ArgumentOfCommandList_t argCmd[CMD_End - 1];
static void ResetFrameData();
static bool IsPassCRC();
void appintf_Init(UART_HandleTypeDef *huart, uint8_t *pTxBuffer, uint8_t txSize, uint8_t *pRxBuffer, uint8_t rxSize) {
	pAppUART = huart;
	_pTxBuffer = pTxBuffer;
	_pRxBuffer = pRxBuffer;
	_txBufSize = txSize;
	_rxBufSize = rxSize;
	HAL_UART_Receive_IT(huart, _pRxBuffer, APP_DATA_LENGTH);
}

void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist) {
	if(cmdlist == CMD_Start || cmdlist == CMD_End) return;
	argCmd[cmdlist].pArg = arg;
	argCmd[cmdlist].sizeArgument = sizeOfArgument;
}

void appintf_RegisterReceivedCallbackEvent(void (*pCpltCallback)(CommandList cmdlist)) {
	pCallback = pCpltCallback;
}

void appintf_RegisterErrorCallbackEvent(void (*pErrorCallback)(AppErrorCode err)) {
	pAppErr = pErrorCallback;
}

void appintf_GetValueFromPayload() {
	if(fd.payloadLength != argCmd[fd.cmdList].sizeArgument)
		if(pAppErr != NULL)
			pAppErr(APPERR_PAYLOAD_NOT_RECOGNIZE);
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	if(argCmd[fd.cmdList].pArg != NULL)
		memcpy(argCmd[fd.cmdList].pArg, _pRxBuffer + payloadField, fd.payloadLength); //offset to payloadField in receive buffer
}


static void DecodeFrameData() {
	uint32_t crcField = APP_DATA_LENGTH + APP_COMMAND_LIST_LENGTH + fd.payloadLength;
	uint32_t crcNibbleByteMSB = *(_pRxBuffer + crcField) << 8;
	uint32_t crcNibbleByteLSB = *(_pRxBuffer + crcField + 1);
	fd.cmdList = *(_pRxBuffer + 1);
	fd.crc16 = crcNibbleByteMSB | crcNibbleByteLSB;
	if(!IsPassCRC())
		if(pAppErr != NULL)
			pAppErr(APPERR_CRC_FAIL);

}

/**
 * @brief Handle frame data received, must be placed inside <void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)> function
 * @example
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 * 		appintf_ReceiveDataInterrupt(huart);
 * }
 * @param huart
 */
void appintf_ReceiveDataInterrupt(UART_HandleTypeDef *huart) {
	if(huart != pAppUART) return;
	if(pAppErr != NULL && pAppUART == NULL)
			pAppErr(APPERR_UART_PORT_NULL);
	static bool isOnFrameReceived = false;
	if(fd.isOnProcess) return;
	// if this is a new frame data, extract data length and receive all remain data in interrupt indicated by data length
	if(!isOnFrameReceived) {
		fd.isOnProcess = true;
		fd.payloadLength = *(_pRxBuffer + 0);
		uint32_t totalLength = fd.totalLength = fd.payloadLength + APP_CRC_LENGTH + APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
		if(totalLength < _rxBufSize)
			HAL_UART_Receive_IT(pAppUART, (uint8_t*) _pRxBuffer + APP_DATA_LENGTH, totalLength - APP_DATA_LENGTH);
		else
		if(pAppErr != NULL) pAppErr(APPERR_OUT_OF_BUFFER_SIZE);
		isOnFrameReceived = true;
		fd.isOnProcess = false;
	}
	else { // if all data of the frame have been received, begin to extract data and reset to receive new frame, and calling to user callback function
		fd.isOnProcess = true;
		isOnFrameReceived = false;
		DecodeFrameData();
		if(pCallback != NULL)
			pCallback(fd.cmdList);
		else
			pAppErr(APPERR_NULL_CALLBACK_FUNCTION);
		HAL_UART_Receive_IT(pAppUART, (uint8_t*) _pRxBuffer, APP_DATA_LENGTH);
		memset(_pRxBuffer, 0, _rxBufSize);
		fd.isOnProcess = false;
		ResetFrameData();
	}

}



AppErrorCode appintf_MakeFrame(CommandList cmdlist) {
	uint32_t totalLength = fd.totalLength = APP_DATA_LENGTH
			+ APP_COMMAND_LIST_LENGTH
			+ argCmd[cmdlist].sizeArgument
			+ APP_CRC_LENGTH;
	if(totalLength > _txBufSize)
		return APPERR_OUT_OF_BUFFER_SIZE;
	if(!argCmd[cmdlist].pArg) return APPERR_STORE_BUFFER_IS_NULL;
	if(!isTxBufferEmpty) memset(_pTxBuffer, 0, _txBufSize);
	fd.isOnProcess = true;
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint16_t crcResult;
	uint32_t crcField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH + argCmd[cmdlist].sizeArgument;
	// Get data length of its command list and load to payloadLength
	fd.payloadLength = argCmd[cmdlist].sizeArgument;
	// Calculate CRC16 from data of its command list (exclude byte cmdlist)
	crcResult = crc16_Unreflected((uint8_t*) argCmd[cmdlist].pArg, argCmd[cmdlist].sizeArgument, 0);
	// Slit CRC16 to 2 bytes to copy to the last 2 bytes of frame data
	uint8_t temp[2] = {crcResult >> 8, crcResult & 0xff};
	// The first byte is data length
	*(_pTxBuffer + 0) = argCmd[cmdlist].sizeArgument;
	// The second byte is command list
	*(_pTxBuffer + 1) = (uint8_t) cmdlist;
	// Remain bytes are payload data
	memcpy(_pTxBuffer + payloadField, argCmd[cmdlist].pArg, argCmd[cmdlist].sizeArgument);
	// The last 2 bytes is CRC16
	memcpy(_pTxBuffer + crcField, temp, 2);
	isTxBufferEmpty = false;
	return APPERR_OK;
}

void appintf_SendFrame() {
	if(!fd.isOnProcess) return;
	HAL_UART_Transmit(pAppUART, _pTxBuffer, fd.totalLength, 10);
	memset(_pTxBuffer, 0, _txBufSize);
	isTxBufferEmpty = true;
	ResetFrameData();
}

static void ResetFrameData() {
	FrameData fdTemp = {0};
	fd = fdTemp;
}

static bool IsPassCRC() {
	uint8_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint8_t *pPayloadCRC = _pRxBuffer + payloadField;
	if(crc16_Unreflected(pPayloadCRC, fd.payloadLength + APP_CRC_LENGTH, 0)) {
		return false;
	}
	return true;
}

