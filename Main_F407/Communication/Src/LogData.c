/*
 * LogData.c
 *
 *  Created on: Mar 14, 2024
 *      Author: SpiritBoi
 */


#include "LogData.h"
#include "stdbool.h"

char txLogBuffer[100] = {0};
UART_HandleTypeDef *targetUART;
bool sendDataComplete = true;

static HAL_StatusTypeDef checkNull(){
	if(targetUART != NULL) return HAL_OK;
	return HAL_ERROR;
}

void log_Init(UART_HandleTypeDef *huart){

	targetUART = huart;
}

void log_TransmitCompleteHandle(UART_HandleTypeDef *huart){
	if(huart != targetUART) return;
	sendDataComplete = true;
	memset(txLogBuffer,0,strlen(txLogBuffer));
}

void log_SendString(){
	if(checkNull() != HAL_OK) return;
	txLogBuffer[strlen(txLogBuffer) -1] = '\n';  //xóa dấu phẩy cuối cùng, thay bằng \n
	if(sendDataComplete) {
		HAL_UART_Transmit_IT(targetUART, (uint8_t*) txLogBuffer, strlen(txLogBuffer));
		sendDataComplete = false;
	}
}


void log_AddHeaderArgumentToBuffer(char *name){
	char s[10] = {0};
	if(strlen(name) > sizeof(s)) return;
	strcpy(s,name);
	sprintf(txLogBuffer+strlen(txLogBuffer),"%s,",s);
}

void log_AddArgumentToBuffer(void *data, LogDataType type) {
	switch(type){
	case TYPE_FLOAT:
		float *fData = (float*)data;
		sprintf(txLogBuffer+strlen(txLogBuffer),"%.2f,",*fData);
		break;
	case TYPE_UINT8:
		uint8_t *uData = (uint8_t*)data;
		sprintf(txLogBuffer+strlen(txLogBuffer),"%u,",*uData);
		break;
	case TYPE_INT:
		int *iData = (int*)data;
		sprintf(txLogBuffer+strlen(txLogBuffer),"%d,",*iData);
		break;
	case TYPE_UINT32:
		uint32_t *u32Data = (uint32_t*)data;
		sprintf(txLogBuffer+strlen(txLogBuffer),"%lu,",*u32Data);
		break;
	}
}
