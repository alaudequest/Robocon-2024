/*
 * AppInterface.h
 *
 *  Created on: Jan 24, 2024
 *      Author: KHOA
 */

#ifndef INC_APPINTERFACE_H_
#define INC_APPINTERFACE_H_
#include "main.h"
#include "string.h"
typedef union FloatByteConvert {
	float f;
	uint8_t b[4];
} FloatByteConvert;

void appinf_Init(UART_HandleTypeDef *huart);
char* appinf_GetBufferAddress();
void appintf_SendFloat(float num);
void appinf_HandleReceive(UART_HandleTypeDef *huart);
#endif /* INC_APPINTERFACE_H_ */
