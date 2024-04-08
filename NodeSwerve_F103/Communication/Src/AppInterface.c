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
#if defined BOARD_SWERVE
ArgumentOfCommandList_t argCmd[CMD_Swerve_End - 1];
#elif defined BOARD_MAINF4_ROBOT1
ArgumentOfCommandList_t argCmd[CMD_MainF4_RB1_End - 1];
#elif defined BOARD_MAINF4_ROBOT2
ArgumentOfCommandList_t argCmd[CMD_MainF4_RB2_End - 1];
#error "You should define which board should use"
#endif
static void ResetFrameData();
static bool IsPassCRC();
static void DecodeFrameDataAndCheckCRC();
static inline void jumpToError(AppErrorCode err);

void appintf_Init(UART_HandleTypeDef *huart, uint8_t *pTxBuffer, uint8_t txSize, uint8_t *pRxBuffer, uint8_t rxSize) {
	pAppUART = huart;
	_pTxBuffer = pTxBuffer;
	_pRxBuffer = pRxBuffer;
	_txBufSize = txSize;
	_rxBufSize = rxSize;
	HAL_UART_Receive_IT(huart, _pRxBuffer, APP_DATA_LENGTH);
}

void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist) {
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
	if (fd.payloadLength != argCmd[fd.cmdList].sizeArgument)
		jumpToError(APPERR_PAYLOAD_NOT_RECOGNIZE);
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	if (argCmd[fd.cmdList].pArg != NULL)
		memcpy(argCmd[fd.cmdList].pArg, _pRxBuffer + payloadField, fd.payloadLength); //offset to payloadField in receive buffer
}

void appintf_GetValueFromPayload_2(void *outData, uint8_t sizeData) {
	if (fd.payloadLength != sizeData)
		jumpToError(APPERR_PAYLOAD_NOT_RECOGNIZE);
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	if (outData != NULL)
		memcpy(outData, _pRxBuffer + payloadField, fd.payloadLength); //offset to payloadField in receive buffer
	else {
		if (pAppErr != NULL)
			pAppErr(APPERR_STORE_BUFFER_IS_NULL);
		else
			return;
	}
}

static void DecodeFrameDataAndCheckCRC() {
	uint32_t crcField = APP_DATA_LENGTH + APP_COMMAND_LIST_LENGTH + fd.payloadLength;
	uint32_t crcNibbleByteMSB = *(_pRxBuffer + crcField) << 8;
	uint32_t crcNibbleByteLSB = *(_pRxBuffer + crcField + 1);
	fd.cmdList = *(_pRxBuffer + 1);
	fd.crc16 = crcNibbleByteMSB | crcNibbleByteLSB;
	if (!IsPassCRC()) {
		jumpToError(APPERR_CRC_FAIL);
	}

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
	if (huart != pAppUART)
		return;
	if (pAppUART == NULL) {
		if (pAppErr != NULL)
			pAppErr(APPERR_UART_PORT_NULL);
		else
			while (1)
				;
	}
	static bool isOnFrameReceived = false;
	if (fd.isOnProcess)
		return;
	// if this is a new frame data, extract data length and receive all remain data in interrupt indicated by data length
	if (!isOnFrameReceived) {
		fd.isOnProcess = true;
		fd.payloadLength = *(_pRxBuffer + 0);
		uint32_t totalLength = fd.totalLength = fd.payloadLength + APP_CRC_LENGTH + APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
		if (totalLength < _rxBufSize)
			HAL_UART_Receive_IT(pAppUART, (uint8_t*) _pRxBuffer + APP_DATA_LENGTH, totalLength - APP_DATA_LENGTH);
		else {
			if (pAppErr != NULL)
				pAppErr(APPERR_OUT_OF_BUFFER_SIZE);
			else
				while (1)
					;
		}
		isOnFrameReceived = true;
		fd.isOnProcess = false;
	}
	else { // if all data of the frame have been received, begin to extract data and reset to receive new frame, and calling to user callback function
		fd.isOnProcess = true;
		isOnFrameReceived = false;
		DecodeFrameDataAndCheckCRC();
		if (pCallback != NULL)
			pCallback(fd.cmdList);
		else {
			if (pAppErr != NULL)
				pAppErr(APPERR_NULL_CALLBACK_FUNCTION);
			else
				return;
		}
		HAL_UART_Receive_IT(pAppUART, (uint8_t*) _pRxBuffer, APP_DATA_LENGTH);
		memset(_pRxBuffer, 0, _rxBufSize);
		fd.isOnProcess = false;
		ResetFrameData();
	}
}

void appintf_MakeFrame(CommandList cmdlist) {
	jumpToError(APPERR_OUT_OF_BUFFER_SIZE);
	if (!argCmd[cmdlist].pArg)
		jumpToError(APPERR_REFERENCE_PAYLOAD_NOT_FOUND);
	if (!isTxBufferEmpty)
		memset(_pTxBuffer, 0, _txBufSize);
	fd.isOnProcess = true;
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint16_t crcResult;
	uint32_t crcField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH + argCmd[cmdlist].sizeArgument;
	// Get data length of its command list and load to payloadLength
	fd.payloadLength = argCmd[cmdlist].sizeArgument;
	// Calculate CRC16 from data of its command list (exclude byte cmdlist)
	crcResult = crc16_Unreflected((uint8_t*) argCmd[cmdlist].pArg, argCmd[cmdlist].sizeArgument, 0);
	// Slit CRC16 to 2 bytes to copy to the last 2 bytes of frame data
	uint8_t temp[2] = { crcResult >> 8, crcResult & 0xff };
	// The first byte is data length
	*(_pTxBuffer + 0) = argCmd[cmdlist].sizeArgument;
	// The second byte is command list
	*(_pTxBuffer + 1) = (uint8_t) cmdlist;
	// Remain bytes are payload data
	memcpy(_pTxBuffer + payloadField, argCmd[cmdlist].pArg, argCmd[cmdlist].sizeArgument);
	// The last 2 bytes is CRC16
	memcpy(_pTxBuffer + crcField, temp, 2);
	isTxBufferEmpty = false;
}

void appintf_MakeFrame_2(void *payloadData, uint8_t sizeOfPayloadData, CommandList cmdlist) {
	uint32_t totalLength = fd.totalLength = APP_DATA_LENGTH
			+ APP_COMMAND_LIST_LENGTH
			+ sizeOfPayloadData
			+ APP_CRC_LENGTH;
	if (totalLength > _txBufSize)
		jumpToError(APPERR_OUT_OF_BUFFER_SIZE);
	if (payloadData == NULL)
		jumpToError(APPERR_REFERENCE_PAYLOAD_NOT_FOUND);
	if (!isTxBufferEmpty)
		memset(_pTxBuffer, 0, _txBufSize);
	fd.isOnProcess = true;
	uint32_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint16_t crcResult;
	uint32_t crcField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH + sizeOfPayloadData;
	fd.payloadLength = sizeOfPayloadData;
	crcResult = crc16_Unreflected((uint8_t*) payloadData, sizeOfPayloadData, 0);
	uint8_t temp[2] = { crcResult >> 8, crcResult & 0xff };
	*(_pTxBuffer + 0) = sizeOfPayloadData;
	*(_pTxBuffer + 1) = (uint8_t) cmdlist;
	memcpy(_pTxBuffer + payloadField, payloadData, sizeOfPayloadData);
	memcpy(_pTxBuffer + crcField, temp, 2);
	isTxBufferEmpty = false;
}

static inline void jumpToError(AppErrorCode err)
{
	if (pAppErr != NULL)
		pAppErr(err);
	else
		while (1);
}

void appintf_SendFrame() {
	if (!fd.isOnProcess)
		return;
	HAL_UART_Transmit(pAppUART, _pTxBuffer, fd.totalLength, 10);
	memset(_pTxBuffer, 0, _txBufSize);
	isTxBufferEmpty = true;
	ResetFrameData();
}

static void ResetFrameData() {
	FrameData fdTemp = { 0 };
	fd = fdTemp;
}

static bool IsPassCRC() {
	uint8_t payloadField = APP_COMMAND_LIST_LENGTH + APP_DATA_LENGTH;
	uint8_t *pPayloadCRC = _pRxBuffer + payloadField;
	if (crc16_Unreflected(pPayloadCRC, fd.payloadLength + APP_CRC_LENGTH, 0)) {
		return false;
	}
	return true;
}

