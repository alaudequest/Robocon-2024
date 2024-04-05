/*
 * MainF4Robot1App.c
 *
 *  Created on: Apr 4, 2024
 *      Author: SpiritBoi
 */

#include <MainF4Robot1App.h>
#include "RB1ActuatorValve.h"
#include "Flag.h"

BoardID brdID = 0;
uint8_t txBuffer[80] = { 0 };
uint8_t rxBuffer[80] = { 0 };
uint8_t relayCommand = 0;
uint8_t valveOutputValue = 0;
extern UART_HandleTypeDef huart2;
static void MainF4Robot1App_ErrorHandler(AppErrorCode err);
static void MainF4Robot1App_ReceiveCommandHandler(CommandList cmdlist);

void MainF4Robot1App_Init()
{
	if (brdID != 0)
		return;
	brdID = BOARD_MainF4_RB1;
	appintf_Init(&huart2, txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer));
	appintf_RegisterErrorCallbackEvent(&MainF4Robot1App_ErrorHandler);
	appintf_RegisterReceivedCallbackEvent(&MainF4Robot1App_ReceiveCommandHandler);
	appintf_RegisterArgument((void*) &brdID, sizeof(brdID), CMD_MainF4_RB1_IdentifyBoard);
	appintf_RegisterArgument((void*) &relayCommand, sizeof(relayCommand), CMD_MainF4_RB1_RelayCommand);
	appintf_RegisterArgument((void*) &valveOutputValue, sizeof(valveOutputValue), CMD_MainF4_RB1_Valve);
}

static void RelayCommandHandler()
{

}

static void MainF4Robot1App_ReceiveCommandHandler(CommandList cmdlist)
{
	switch (cmdlist) {
		case CMD_MainF4_RB1_IdentifyBoard:
			appintf_MakeFrame(cmdlist);
			appintf_SendFrame();
			break;
		case CMD_MainF4_RB1_Valve:
			appintf_GetValueFromPayload();
			valve_OutputAllPin(valveOutputValue);
			break;
		case CMD_MainF4_RB1_RelayCommand:
			RelayCommandHandler();
			break;
		default:
			MainF4Robot1App_ErrorHandler(APPERR_BOARD_FEATURE_NOT_SUPPORT);
			break;
	}
}

static void MainF4Robot1App_ErrorHandler(AppErrorCode err)
{
	while (1);
}

static inline void Convert_AppParamPID_to_PIDParam(AppPararmPID_t appPID, PID_Param *pid) {
	pid->kP = appPID.kp;
	pid->kI = appPID.ki;
	pid->kD = appPID.kd;
	pid->alpha = appPID.alpha;
	pid->deltaT = appPID.deltaT;
	pid->u_AboveLimit = appPID.limitHigh;
	pid->u_BelowLimit = appPID.limitLow;
}

static inline void Convert_PIDParam_to_AppParamPID(AppPararmPID_t *appPID, PID_Param pid) {
	appPID->kp = pid.kP;
	appPID->ki = pid.kI;
	appPID->kd = pid.kD;
	appPID->alpha = pid.alpha;
	appPID->deltaT = pid.deltaT;
	appPID->limitHigh = pid.u_AboveLimit;
	appPID->limitLow = pid.u_BelowLimit;

}

