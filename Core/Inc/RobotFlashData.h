/*
 * RobotFlashData.h
 *
 *  Created on: Apr 28, 2024
 *      Author: ADMIN
 */

#ifndef INC_ROBOTFLASHDATA_H_
#define INC_ROBOTFLASHDATA_H_

#include "main.h"
#include "BoardParameter.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"

#define FLASH_ROBOT_TARGET_ADDRESS 0x080e0000


typedef struct Robot1FlashData_t{
	TeamColor teamColor;
	PaddyRice_t riceCollected;
}Robot1FlashData_t;

typedef struct Robot2FlashData_t{
	TeamColor teamColor;
	HarvestZoneBallPosition_t ballPosition;
	CustomGamepad_t customGamepad;
	uint8_t totalBallSuccess;
}Robot2FlashData_t;

void RBFlash_EndOperationHandler(uint32_t ReturnValue);
void RBFlash_ErrorHandler(uint32_t ReturnValue);
void RBFlash_WriteTeamColor(TeamColor color);
TeamColor RBFlash_GetTeamColor();
bool RBFlash_IsProgramComplete();
void RBFlash_WriteToFlash(void *pData, uint8_t sizeOfData);
void RBFlash_Test();
#endif /* INC_ROBOTFLASHDATA_H_ */
