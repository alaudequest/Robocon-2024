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

#ifdef BOARD_SWERVE
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
	CMD_End,	//not use
} CommandList;

typedef enum NodeSwerveRelayCommand {
	RunBLDC = 1,
	RunDC,
	InverseDirection,
} NodeSwerveRelayCommand;
#elif BOARD_MAIN
typedef enum CommandList {
	CMD_Start = 0, //not use
	CMD_IdentifyBoard,
	CMD_GetPID,
	CMD_SetPID,
	CMD_SavePID,
	CMD_TriggerValve,
	CMD_RelayCommand,
	CMD_Process_SetProcessStep,
	CMD_Process_SetVelocity,
	CMD_Process_SetDistance,
	CMD_Process_SetAngle,
	CMD_NotFound,
	CMD_Busy,
	CMD_End,	//not use
} CommandList;

typedef enum MainRelayCommand {
	RB1_Arm1Catch= 1,
	RB1_Arm2Catch,
	RB1_CollectBallLeft,
	RB1_CollectBallRight,
} NodeSwerveRelayCommand;
#else
#error "You must define which board should use"
#endif


typedef enum BoardID {
	BOARD_MainF4_RB1 = 1,
	BOARD_MainF4_RB2,
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
	APPERR_BOARD_NOT_FOUND,
	APPERR_COMMAND_NOT_FOUND,
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

typedef struct ArgumentOfCommandList_t {
	void *pArg;
	uint8_t sizeArgument;
} ArgumentOfCommandList_t;
typedef void (*pCpltCallback)(CommandList cmdlist);
typedef void (*pErrorCallback)(AppErrorCode err);

void appintf_Init(UART_HandleTypeDef *huart, uint8_t *pTxBuffer, uint8_t txSize, uint8_t *pRxBuffer, uint8_t rxSize);
void appintf_ReceiveDataInterrupt(UART_HandleTypeDef *huart);
void appintf_ErrorHandler(AppErrorCode err);
void appintf_RegisterReceivedCallbackEvent(void (*pCpltCallback)(CommandList cmdlist));
void appintf_RegisterErrorCallbackEvent(void (*pErrorCallback)(AppErrorCode err));
void appintf_SendFrame();
AppErrorCode appintf_MakeFrame(CommandList cmdlist);
void appintf_GetValueFromPayload();
void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist);

#endif /* INC_APPINTERFACE_H_ */
