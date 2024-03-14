/*
 * LogData.h
 *
 *  Created on: Mar 14, 2024
 *      Author: Admin
 */

#ifndef INC_LOGDATA_H_
#define INC_LOGDATA_H_
#include "main.h"
#include "string.h"
#include "stdio.h"

typedef enum DataType{
	TYPE_UINT8,
	TYPE_INT,
	TYPE_FLOAT,
	TYPE_UINT32,
}DataType;


void log_Init(UART_HandleTypeDef *huart);
void log_TransmitCompleteHandle(UART_HandleTypeDef *huart);
void log_AddArgumentToBuffer(void *data, DataType type);
void log_AddHeaderArgumentToBuffer(char *name);
void log_SendString();


#endif /* INC_LOGDATA_H_ */
