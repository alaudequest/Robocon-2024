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
	CMD_Swerve_Start = 0, //not use
	CMD_Swerve_IdentifyBoard,
	CMD_Swerve_GetPID,
	CMD_Swerve_SetPID,
	CMD_Swerve_SavePID,
	CMD_Swerve_RelayCommand,
	CMD_Swerve_SetTargetSpeedBLDC,
	CMD_Swerve_SetTargetAngleDC,
	CMD_Swerve_GetCurrentSpeedBLDC,
	CMD_Swerve_GetCurrentAngleDC,
	CMD_Swerve_UntangleBLDC,
	CMD_Swerve_SetHome,
	CMD_Swerve_End, //not use
	CMD_MainF4_RB1_Start = 0, //not use
	CMD_MainF4_RB1_IdentifyBoard,
	CMD_MainF4_RB1_Valve,
	CMD_MainF4_RB1_RelayCommand,
	CMD_MainF4_RB1_Process_SetProcessStep,
	CMD_MainF4_RB1_Process_SetVelocity,
	CMD_MainF4_RB1_Process_SetDistance,
	CMD_MainF4_RB1_Process_SetAngle,
	CMD_MainF4_RB1_End,	//not use
	CMD_MainF4_RB2_Start = 0,
	CMD_MainF4_RB2_End,
} CommandList;

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

typedef enum AppErrorCode {
	APPERR_OK,
	APPERR_BOARD_NOT_FOUND,
	APPERR_BOARD_FEATURE_NOT_SUPPORT,
	APPERR_COMMAND_NOT_FOUND,
	APPERR_OUT_OF_COMMAND_LIST,
	APPERR_NULL_CALLBACK_FUNCTION,
	APPERR_CRC_FAIL,
	APPERR_FRAME_ERROR,
	APPERR_OUT_OF_BUFFER_SIZE,
	APPERR_UART_PORT_NULL,
	APPERR_SEND_FRAME_FAIL,
	APPERR_REFERENCE_PAYLOAD_NOT_FOUND,
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
void appintf_Reset();
void appintf_ReceiveDataInterrupt(UART_HandleTypeDef *huart);
void appintf_ErrorHandler(AppErrorCode err);
void appintf_RegisterReceivedCallbackEvent(void (*pCpltCallback)(CommandList cmdlist));
void appintf_RegisterErrorCallbackEvent(void (*pErrorCallback)(AppErrorCode err));
void appintf_SendFrame();
void appintf_MakeFrame(CommandList cmdlist);
void appintf_MakeFrame_2(void *payloadData, uint8_t sizeOfPayloadData, CommandList cmdlist);
void appintf_GetValueFromPayload();
void appintf_GetValueFromPayload_2(void *outData, uint8_t sizeData);
void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist);

#endif /* INC_APPINTERFACE_H_ */
