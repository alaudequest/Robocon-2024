/*
 * UART_Utility.c
 *
 *  Created on: Apr 17, 2023
 *      Author: SpiritBoi
 */


#include "UART_Utility.h"
#ifdef CONFIG_USE_UART_UTILITY

UART_Utility_t *_util;

void UART_Util_BeginToGetMessage(UART_Utility_t *util,UART_HandleTypeDef *huart,uint8_t *MesgBuffer,char *CharEndOfMessage)
{
	if(!util) return;
	_util = util;
	UART = huart;
	charEndOfMessage = CharEndOfMessage;
	utilBuf = MesgBuffer;
	HAL_UART_Receive_IT(huart, utilBufTemp, 1);
}

void UART_Util_SetTarget(UART_Utility_t *util)
{
	if(!util) return;
	_util = util;
}

void UART_Util_GetMessage_IT_Callback(UART_Utility_t *util, UART_HandleTypeDef *huart){
	static uint8_t count=0;
	if(util) _util=util;
	if(huart->Instance == UART->Instance)
	{
		HAL_UART_Receive_IT(UART, utilBufTemp, 1);
		count++;
		strcat((char*)utilBuf,(char*)utilBufTemp);
		if(!strcmp((char*)utilBufTemp,charEndOfMessage)){
			SETFLAG(utilFlag,UART_UTIL_FLAG_MESSAGE_GET_COMPLETE);
		}
	}
}

bool UART_Util_CheckGetMessageComplete(UART_Utility_t *util, bool ClearAfterCheck)
{
	if(util) _util=util;
	if(ClearAfterCheck){
		uint8_t a = CHECKFLAG(utilFlag,UART_UTIL_FLAG_MESSAGE_GET_COMPLETE);
		if (a) CLEARFLAG(utilFlag,UART_UTIL_FLAG_MESSAGE_GET_COMPLETE);
		return a;

	}
	return CHECKFLAG(utilFlag,UART_UTIL_FLAG_MESSAGE_GET_COMPLETE);
}



#endif



