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

#define COMMAND_LIST_LENGTH 1
#define CRC16_LEN	2

typedef union FloatByteConvert {
	float f;
	uint8_t b[4];
} FloatByteConvert;

typedef enum CommandList {
	CMD_Start,
	CMD_SetpointX,
	CMD_SetpointY,
	CMD_SetpointTheta,
	CMD_TimeStableX,
	CMD_TimeStableY,
	CMD_TimeStableTheta,
	CMD_OffsetX,
	CMD_OffsetY,
	CMD_OffsetTheta,
	CMD_CurrentState,
	CMD_MainPID,
	CMD_End,
} CommandList;

typedef struct MainF407_ParamPID {
	float kp;
	float kd;
	float alpha;
	float limitHigh;
	float limitLow;
	float offsetSteady;
} MainF407_ParamPID;

typedef struct AppProtocol {
	uint8_t dataLen;
	uint16_t crc16;
	CommandList cmdList;
	uint8_t *data;
} AppProtocol;
void appinf_Init(UART_HandleTypeDef *huart);
char* appinf_GetBufferAddress();
void appintf_SendFloat(float num);
void appinf_HandleReceive(UART_HandleTypeDef *huart);
#endif /* INC_APPINTERFACE_H_ */
