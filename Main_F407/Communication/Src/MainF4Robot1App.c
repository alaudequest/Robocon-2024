/*
 * MainF4Robot1App.c
 *
 *  Created on: Apr 4, 2024
 *      Author: SpiritBoi
 */

#include <MainF4Robot1App.h>
#include "RB1ActuatorValve.h"

BoardID brdID = BOARD_MainF4_RB1;
uint8_t txBuffer[80] = { 0 };
uint8_t rxBuffer[80] = { 0 };
uint8_t relayCommand = 0;
extern UART_HandleTypeDef huart2;
static void MainF4Robot1App_ErrorHandler(AppErrorCode err);
static void MainF4Robot1App_ReceiveCommandHandler(CommandList cmdlist);

void MainF4Robot1App_Init() {
	appintf_Init(&huart2, txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer));
	appintf_RegisterErrorCallbackEvent(&MainF4Robot1App_ErrorHandler);
	appintf_RegisterReceivedCallbackEvent(&MainF4Robot1App_ReceiveCommandHandler);
	appintf_RegisterArgument((void*) &brdID, sizeof(brdID), CMD_MainF4_RB1_IdentifyBoard);
	appintf_RegisterArgument((void*) &relayCommand, sizeof(relayCommand), CMD_MainF4_RB1_RelayCommand);
}

static void MainF4Robot1App_ErrorHandler(AppErrorCode err)
{
	while (1);
}

static void MainF4Robot1App_ReceiveCommandHandler(CommandList cmdlist)
{
	switch (cmdlist) {

		default:
			MainF4Robot1App_ErrorHandler(APPERR_BOARD_FEATURE_NOT_SUPPORT);
			break;
	}
}
