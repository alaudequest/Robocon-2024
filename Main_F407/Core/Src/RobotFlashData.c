/*
 * RobotFlashData.c
 *
 *  Created on: Apr 29, 2024
 *      Author: SpiritBoi
 */

/*
 * RobotFlashData.c
 *
 *  Created on: Apr 28, 2024
 *      Author: ADMIN
 */

#include "RobotFlashData.h"
#include "BoardParameter.h"


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

void RBFlash_WriteToFlash(void *pData, uint8_t sizeOfData, uint32_t address)
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
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + byteCompleted, Data);
		}
		else if (byteRemain >= 2) {
			memcpy(&Data, pData + byteCompleted, 2);
			currentTypeProgram = FLASH_TYPEPROGRAM_HALFWORD;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + byteCompleted, Data);
		}
		else {
			memcpy(&Data, pData + byteCompleted, 1);
			currentTypeProgram = FLASH_TYPEPROGRAM_BYTE;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + byteCompleted, Data);
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

void RBFlash_ReadDataFromFlash(void *pData, uint8_t sizeOfData, uint32_t address)
{

	// Copy data từ flash vào cấu trúc dữ liệu được chỉ định, mỗi bước nhảy địa chỉ để lấy giá trị từ đó là 1 byte nên dùng (uint8_t*)
	memcpy(pData, (uint8_t*)address, sizeOfData);
}
void RBFlash_ErrorHandler(uint32_t ReturnValue)
{

}

