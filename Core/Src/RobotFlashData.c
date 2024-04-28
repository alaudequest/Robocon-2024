/*
 * RobotFlashData.c
 *
 *  Created on: Apr 28, 2024
 *      Author: ADMIN
 */

#include "RobotFlashData.h"
#include "BoardParameter.h"

static bool isGetDataFromFlash = false;

#if defined(BOARD_MAINF4_ROBOT1)
Robot1FlashData_t rb1FlashData;
#elif defined(BOARD_MAINF4_ROBOT2)
Robot2FlashData_t rb2FlashData;
#endif

static void EraseFlash()
{
	FLASH_EraseInitTypeDef fe;
	fe.TypeErase = FLASH_TYPEERASE_SECTORS;
	fe.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	fe.Sector = FLASH_SECTOR_11;
	fe.NbSectors = 1;
	uint32_t pageErr = 0;
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&fe, &pageErr) != HAL_OK) {
//		 return HAL_FLASH_GetError();
		while (1);
	}
	HAL_FLASH_Lock();
}

void RBFlash_WriteToFlash(void *pData, uint8_t sizeOfData)
{
	EraseFlash();
	HAL_FLASH_Unlock();
	uint64_t Data = 0;
	uint32_t byteCompleted = 0;
	uint32_t currentTypeProgram;
	while (byteCompleted != sizeOfData) {
		uint16_t byteRemain = sizeOfData - byteCompleted;
		if (byteRemain >= 4) {
			memcpy(&Data, pData + byteCompleted, 4);
			currentTypeProgram = FLASH_TYPEPROGRAM_WORD;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ROBOT_TARGET_ADDRESS + byteCompleted, Data);
		}
		else if (byteRemain >= 2) {
			memcpy(&Data, pData + byteCompleted, 2);
			currentTypeProgram = FLASH_TYPEPROGRAM_HALFWORD;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ROBOT_TARGET_ADDRESS + byteCompleted, Data);
		}
		else {
			memcpy(&Data, pData + byteCompleted, 1);
			currentTypeProgram = FLASH_TYPEPROGRAM_BYTE;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ROBOT_TARGET_ADDRESS + byteCompleted, Data);
		}
		switch (currentTypeProgram) {
			case FLASH_TYPEPROGRAM_WORD:
				byteCompleted += 4;
				break;
			case FLASH_TYPEPROGRAM_HALFWORD:
				byteCompleted += 2;
				break;
			case FLASH_TYPEPROGRAM_BYTE:
				byteCompleted += 1;
				break;
		}
	}
	HAL_FLASH_Lock();
}

static void ReadDataFromFlash(void *pData, uint8_t sizeOfData)
{
	if (isGetDataFromFlash)
		return;
	memcpy(pData, (uint8_t*) FLASH_ROBOT_TARGET_ADDRESS, sizeOfData);
	isGetDataFromFlash = true;
}

#ifdef BOARD_MAINF4_ROBOT1


#endif

#ifdef BOARD_MAINF4_ROBOT2

void RBFlash_ErrorHandler(uint32_t ReturnValue)
{

}

void RBFlash_UpdateBoardParametersToFlash()
{
	rb2FlashData.customGamepad = BrdParam_GetCustomGamepad();
	rb2FlashData.teamColor = BrdParam_GetTeamColor();
	rb2FlashData.totalBallSuccess = BrdParam_GetBallSuccess();
//	rb2FlashData.targetBallPosition
	RBFlash_WriteToFlash((void*) &rb2FlashData, sizeof(rb2FlashData));
}

void RBFlash_LoadDataFromFlashToBoardParameters()
{
	ReadDataFromFlash((void*) &rb2FlashData, sizeof(rb2FlashData));
	BrdParam_SetBallSuccess(rb2FlashData.totalBallSuccess);
	BrdParam_SetCustomGamepad(rb2FlashData.customGamepad);
	BrdParam_SetTeamColor(rb2FlashData.teamColor);
}

void RBFlash_Test()
{
	Robot2FlashData_t test;
	test.teamColor = BLUE_TEAM;
	test.customGamepad.ballMatrix[0] = 12;
	test.customGamepad.ballMatrix[1] = 23;
	test.customGamepad.ballMatrix[2] = 34;
	test.customGamepad.ballMatrix[3] = 45;
	test.customGamepad.ballMatrix[4] = 56;
	test.customGamepad.ballMatrix[5] = 67;
	test.targetBallPosition.column = 2;
	test.targetBallPosition.row = 1;
	test.customGamepad.siloNum = 5;
//	RBFlash_WriteToFlash((void*)&test, sizeof(test));
	ReadDataFromFlash((void*) &rb2FlashData, sizeof(rb2FlashData));

}
#endif

