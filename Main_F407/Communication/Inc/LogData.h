/*
 * LogData.h
 *
 *  Created on: Mar 14, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_LOGDATA_H_
#define INC_LOGDATA_H_
#include "main.h"
#include "string.h"
#include "stdio.h"

/*
 * Example code
 *
 *
 * UART_HandleTypeDef huart2;
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
 *  log_TransmitCompleteHandle(huart);
 * }
 *
 * int main(void){
 *		int a = 10;
 *		uint8_t b = 250;
 *		float c = 10.25;
 * 		log_Init(&huart2);
 * 		log_AddHeaderArgumentToBuffer("a");
 * 		log_AddHeaderArgumentToBuffer("b");
 * 		log_AddHeaderArgumentToBuffer("c");
 * 		log_SendString();
 * 		log_AddArgumentToBuffer((void*)&a,TYPE_INT);
 * 		log_AddArgumentToBuffer((void*)&b,TYPE_UINT8);
 * 		log_AddArgumentToBuffer((void*)&c,TYPE_FLOAT);
 * 		log_SendString();
 * }
 *
 * Output result:
 * a,b,c\n
 * 10,250,10.25\n
 */

typedef enum LogDataType {
	TYPE_UINT8,
	TYPE_INT,
	TYPE_FLOAT,
	TYPE_UINT32,
} LogDataType;


void log_Init(UART_HandleTypeDef *huart);
void log_TransmitCompleteHandle(UART_HandleTypeDef *huart);
void log_AddArgumentToBuffer(void *data, DataType type);
void log_AddHeaderArgumentToBuffer(char *name);
void log_SendString();


#endif /* INC_LOGDATA_H_ */
