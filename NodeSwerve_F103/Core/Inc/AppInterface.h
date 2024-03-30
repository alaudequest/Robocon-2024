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


#define APP_COMMAND_LIST_LENGTH 1
#define APP_CRC_LENGTH	2
#define APP_DATA_LENGTH 1
#define APP_BUFFER_SIZE 30

typedef enum Argument_XYTheta {
	APPINTF_ARG_X,
	APPINTF_ARG_Y,
	APPINTF_ARG_THETA,
} Argument_XYTheta;

typedef enum CommandList {
	CMD_Start = 0, //not use
	// each point in TrajectoryPlanning contains one TrajectoryPlanningParameter
	CMD_TrajectoryPlanningPoint = 1,
	// PD algorithm for MainF407
	CMD_MainPD,
	CMD_CurrentXYTheta,
	CMD_Busy,
	CMD_End,	//not use
} CommandList;

typedef enum AppErrorCode {
	APPERR_OK,
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

typedef struct MainF407_ParamPD {
	float kp;
	float kd;
	float alpha;
	float limitHigh;
	float limitLow;
} MainF407_ParamPD;

typedef struct FrameData {
	uint8_t payloadLength;
	uint8_t totalLength;
	uint16_t crc16;
	// avoid critical section by sending and receiving frame in the same FrameData variable
	bool isOnProcess;
	CommandList cmdList;
} FrameData;

typedef struct CoordinateCurrent {
	float currentX;
	float currentY;
	float currentTheta;
} CoordinateCurrent;

typedef struct TrajectoryPlanningParameter {
	float setpointX;
	float setpointY;
	float setpointTheta;
	float tStableX;
	float tStableY;
	float tStableTheta;
	uint8_t currentState;
} TrajectoryPlanningParameter;

typedef struct DataContainer {
	MainF407_ParamPD mpd;
	TrajectoryPlanningParameter traject;
} DataContainer;

typedef void (*pCpltCallback)(CommandList cmdlist);

void appintf_Init(UART_HandleTypeDef *huart);
void appintf_HandleReceive(UART_HandleTypeDef *huart);
void appintf_ErrorHandler(AppErrorCode err);
void appintf_RegisterCallbackEvent(void (*pCpltCallback)(CommandList cmdlist));
void appintf_SendFrame();
AppErrorCode appintf_MakeFrame(CommandList cmdlist);
void appintf_RegisterArgument(void *arg, uint8_t sizeOfArgument, CommandList cmdlist);

#endif /* INC_APPINTERFACE_H_ */
