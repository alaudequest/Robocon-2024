/*
 * UART_Utility.h
 *
 *  Created on: Apr 17, 2023
 *      Author: SpiritBoi
 */

#ifndef UART_UTILITY_H_
#define UART_UTILITY_H_
#include "main.h"
#ifdef CONFIG_USE_UART_UTILITY
#include "string.h"
#include "Flag.h"
#include "stdbool.h"

typedef struct UART_Utility_t {
	UART_HandleTypeDef *huart;
	char *charEndOfMessage;
	uint8_t *buf;
	uint8_t bufTemp[1];
	uint16_t flag;
}UART_Utility_t;
#define UART_UTIL_FLAG_MESSAGE_GET_COMPLETE (1<<0)

#define UART _util->huart
#define charEndOfMessage _util->charEndOfMessage
#define utilBufTemp _util->bufTemp
#define utilBuf _util->buf
#define utilFlag _util->flag

void UART_Util_BeginToGetMessage(UART_Utility_t *util,UART_HandleTypeDef *huart,uint8_t *MesgBuffer,char *CharEndOfMessage);
void UART_Util_GetMessage_IT_Callback(UART_Utility_t *util, UART_HandleTypeDef *huart);
void UART_Util_SetTarget(UART_Utility_t *utillity);
bool UART_Util_CheckGetMessageComplete(UART_Utility_t *util, bool ClearAfterCheck);
#endif
#endif


