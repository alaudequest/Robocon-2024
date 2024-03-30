/*
 * NodeSwerve_AppInterface.c
 *
 *  Created on: Mar 30, 2024
 *      Author: SpiritBoi
 */

#include "NodeSwerve_AppInterface.h"
extern UART_HandleTypeDef huart1;
uint8_t txBuffer[30] = {0};
uint8_t rxBuffer[30] = {0};
BoardID brdID = 0;
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
	appintf_MakeFrame(CMD_IdentifyBoard);
	appintf_SendFrame();
}

void SwerveApp_ErrorHandler(AppErrorCode err)
{
	while (1);
}

static void SendArgumentToApp(CommandList cmdlist) {
	appintf_MakeFrame(cmdlist);
	appintf_SendFrame();
}

void SwerveApp_ReceiveCommandHandler(CommandList cmdlist)
{
	switch (cmdlist) {
		case CMD_IdentifyBoard:
		case CMD_GetPID:
			SendArgumentToApp(cmdlist);
			break;
		case CMD_SetPID:
			case CMD_SetSpeedBLDC:
			case CMD_SetSpeedDC:
			case CMD_SetAngleDC:
			appintf_GetValueFromPayload();
			break;
		case CMD_RelayCommand:
			break;
		default:
			break;
	}
}
