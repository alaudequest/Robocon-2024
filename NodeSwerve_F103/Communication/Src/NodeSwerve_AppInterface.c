/*
 * NodeSwerve_AppInterface.c
 *
 *  Created on: Mar 30, 2024
 *      Author: SpiritBoi
 */

#include "NodeSwerve_AppInterface.h"
extern UART_HandleTypeDef huart1;
uint8_t txBuffer[80] = {0};
uint8_t rxBuffer[80] = {0};
BoardID brdID = 0;
AppPararmPID_t _appPID;

static inline void Convert_AppParamPID_to_PIDParam(AppPararmPID_t appPID, PID_Param *pid);
static inline void Convert_PIDParam_to_AppParamPID(AppPararmPID_t *appPID, PID_Param pid);

void SwerveApp_ErrorHandler(AppErrorCode err);
void SwerveApp_ReceiveCommandHandler(CommandList cmdlist);

void SwerveApp_Init()
{
	// get ID board from flash memory
	uint32_t swerveID = *(__IO uint32_t*) (0x08000000 + 1024 * 64);
	switch (swerveID) {
		case 0:
			SwerveApp_ErrorHandler(APPERR_BOARD_NOT_FOUND);
		break;
		case 1:
			brdID = BOARD_NodeSwerve1;
		break;
		case 2:
			brdID = BOARD_NodeSwerve2;
		break;
		case 3:
			brdID = BOARD_NodeSwerve3;
		break;
	}
	appintf_Init(&huart1, txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer));
	// If any error happen, calling to this function
	appintf_RegisterErrorCallbackEvent(&SwerveApp_ErrorHandler);
	// After receiving data from app, calling to this function to handle command list
	appintf_RegisterReceivedCallbackEvent(&SwerveApp_ReceiveCommandHandler);
	appintf_RegisterArgument((void*) &brdID, sizeof(brdID), CMD_IdentifyBoard);
	appintf_RegisterArgument((void*) &_appPID, sizeof(_appPID), CMD_GetPID);
	appintf_RegisterArgument((void*) &_appPID, sizeof(_appPID), CMD_SetPID);
}

void SwerveApp_ErrorHandler(AppErrorCode err)
{
	while (1);
}

static void SendArgumentToApp(CommandList cmdlist) {
	appintf_MakeFrame(cmdlist);
	appintf_SendFrame();
}

static void HandleCommandSetPID() {
	appintf_GetValueFromPayload();
	PID_Param pid;
	Convert_AppParamPID_to_PIDParam(_appPID, &pid);
	brd_SetPID(pid, _appPID.type);
}

static void HandleCommandGetPID() {

	// vì trên app truyền giá trị byte xuống đại diện cho enum PID_type, nên khi lấy ra cũng phải khớp kiểu uint8_t
	uint8_t temp;
	appintf_GetValueFromPayload_2((void*) &temp, sizeof(temp));
	PID_type typePID = temp;
	PID_Param pid = brd_GetPID(typePID);
	Convert_PIDParam_to_AppParamPID(&_appPID, pid);
	appintf_MakeFrame(CMD_GetPID);
	appintf_SendFrame();
}


void SwerveApp_ReceiveCommandHandler(CommandList cmdlist)
{
	switch (cmdlist) {
		case CMD_IdentifyBoard:
		SendArgumentToApp(cmdlist);
		break;
		case CMD_SetSpeedBLDC:
		case CMD_SetSpeedDC:
		case CMD_SetAngleDC:
		appintf_GetValueFromPayload();
		break;
		case CMD_SetPID:
		HandleCommandSetPID();
		break;
		case CMD_GetPID:
		HandleCommandGetPID();
		break;
		case CMD_RelayCommand:
		break;
		default:
		break;
	}
}

static inline void Convert_AppParamPID_to_PIDParam(AppPararmPID_t appPID, PID_Param *pid)
{
	pid->kP = appPID.kp;
	pid->kI = appPID.ki;
	pid->kD = appPID.kd;
	pid->alpha = appPID.alpha;
	pid->deltaT = appPID.deltaT;
	pid->u_AboveLimit = appPID.limitHigh;
	pid->u_BelowLimit = appPID.limitLow;
}

static inline void Convert_PIDParam_to_AppParamPID(AppPararmPID_t *appPID, PID_Param pid)
{
	appPID->kp = pid.kP;
	appPID->ki = pid.kI;
	appPID->kd = pid.kD;
	appPID->alpha = pid.alpha;
	appPID->deltaT = pid.deltaT;
	appPID->limitHigh = pid.u_AboveLimit;
	appPID->limitLow = pid.u_BelowLimit;

}
