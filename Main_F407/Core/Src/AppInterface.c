/*
 * AppInterface.c
 *
 *  Created on: Jan 24, 2024
 *      Author: SpiritBoi
 */
#include "AppInterface.h"
#include "stdbool.h"

char tempBuffer[1] = {0};
char uartRxData[30] = {0};
UART_HandleTypeDef *pAppUART;
bool isReceivedFrameData = false;

void appinf_Init(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(huart, (uint8_t*) tempBuffer, 1);
	pAppUART = huart;
}

void appintf_SendFloat(float num) {
	FloatByteConvert temp = {
			.f = num,
	};
	HAL_UART_Transmit(pAppUART, temp.b, sizeof(float), 10);
}

char* appinf_GetBufferAddress() {
	return tempBuffer;
}

void appinf_HandleReceive(UART_HandleTypeDef *huart) {
	if (huart != pAppUART) return;
	static uint8_t i = 0;
	if (tempBuffer[0] == '\n') {
		tempBuffer[0] = 0;
		FloatByteConvert temp;
		memcpy(temp.b, uartRxData, sizeof(float));
		i = 0;
		memset(uartRxData, 0, sizeof(uartRxData));
		isReceivedFrameData = true;
	} else {
		memcpy(uartRxData + i, tempBuffer, 1);
		i++;
	}
	HAL_UART_Receive_IT(pAppUART, (uint8_t*) tempBuffer, 1);
}

void appinf_DecodeFrameData() {
	if (!isReceivedFrameData) return;

}

