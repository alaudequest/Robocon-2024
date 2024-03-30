/*
 * AppInterface.h
 *
 *  Created on: Jan 24, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_APPINTERFACE_H_
#define INC_APPINTERFACE_H_
#include "main.h"
#include "string.h"
#include "stdbool.h"
#include "CRC16.h"

typedef enum CommandList {
	CMD_Start = 0, //not use
	CMD_IdentifyBoard,
	CMD_GetPID,
	CMD_SetPID,
	CMD_SavePID,
	CMD_RelayCommand,
	CMD_SetSpeedBLDC,
	CMD_SetSpeedDC,
	CMD_SetAngleDC,
	CMD_NotFound,
	CMD_Busy,
	CMD_End,	//not use
} CommandList;

typedef enum NodeSwerveRelayCommand {
	RunBLDC = 1,
	RunDC,
	InverseDirection,
};

typedef enum BoardID {
	BOARD_MainF4 = 1,
	BOARD_NodeSwerve1,
	BOARD_NodeSwerve2,
	BOARD_NodeSwerve3,
} BoardID;

#define APP_COMMAND_LIST_LENGTH 1
#define APP_CRC_LENGTH	2
#define APP_DATA_LENGTH 1
#define APP_BUFFER_SIZE 30


typedef enum AppErrorCode {
	APPERR_OK,
	APPERR_NOT_FOUND_COMMAND,
	APPERR_OUT_OF_COMMAND_LIST,
	APPERR_NULL_CALLBACK_FUNCTION,
	APPERR_CRC_FAIL,
	APPERR_FRAME_ERROR,
	APPERR_OUT_OF_BUFFER_SIZE,
	APPERR_UART_PORT_NULL,
	APPERR_SEND_FRAME_FAIL,
	APPERR_PAYLOAD_NOT_RECOGNIZE,
	APPERR_STORE_BUFFER_IS_NULL,
	APPERR_END,
} AppErrorCode;

typedef struct AppInterfaceParamPID {
	float kp;
	float ki;
	float kd;
	float alpha;
	float deltaT;
	float limitHigh;
	float limitLow;
} AppInterfaceParamPID;

typedef struct FrameData {
	uint8_t payloadLength;
	uint8_t totalLength;
	uint16_t crc16;
	// avoid critical section by sending and receiving frame in the same FrameData variable
	bool isOnProcess;
	CommandList cmdList;
} FrameData;

typedef void (*pCpltCallback)(CommandList cmdlist);
typedef void (*pErrorCallback)(AppErrorCode err);

void appintf_Init(UART_HandleTypeDef *huart);
void appintf_HandleReceive(UART_HandleTypeDef *huart);
void appintf_ErrorHandler(AppErrorCode err);
void appintf_RegisterReceivedCallbackEvent(void (*pCpltCallback)(CommandList cmdlist));
void appintf_RegisterErrorCallbackEvent(void (*pErrorCallback)(AppErrorCode err));
void appintf_SendFrame();
AppErrorCode appintf_MakeFrame(CommandList cmdlist);
void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist);

#endif /* INC_APPINTERFACE_H_ */
