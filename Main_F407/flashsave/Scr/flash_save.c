/*
 * flash_save.c
 *
 *  Created on: Sep 29, 2023
 *      Author: NamDHay
 */

#include "flash_save.h"
#include "main.h"

void Flash_Erase(uint32_t sector)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef fe;
	fe.TypeErase = FLASH_TYPEERASE_SECTORS;
	fe.Sector = sector;
	fe.NbSectors = 1;
	fe.Banks = FLASH_BANK_1;
	fe.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	uint32_t sectorErr = 0;
	if(HAL_FLASHEx_Erase(&fe, &sectorErr) != HAL_OK){
		while(1);
	}
	HAL_FLASH_Lock();
}

HAL_StatusTypeDef Flash_Write(uint32_t address, void *data, size_t sizeofDataType){
	static bool IsBusy = false;
	static uint8_t lengh = 0;
	static uint32_t addr = 0;
	uint16_t *dta = NULL;
	static uint16_t *pdta = NULL;
	if(IsBusy)	return HAL_BUSY;
	if(!address)	return HAL_ERROR;
	if(!lengh && !addr){
		lengh = sizeofDataType;
		addr = address;
	}
	if(!pdta){
		dta = (uint16_t*)data;
	}else
		dta = pdta;
	HAL_FLASH_Unlock();
	IsBusy = true;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, *dta) == HAL_OK){
		IsBusy = false;
		lengh -= 2;
		addr += 2;
		dta++;
		pdta = dta;
	}
	if(lengh >= 1 && lengh <= sizeofDataType){
		return HAL_BUSY;
	}else
		return HAL_OK;
	HAL_FLASH_Lock();
	return HAL_BUSY;
}

HAL_StatusTypeDef Flash_Read(uint32_t address, void *data, size_t sizeOfDataType){
	static uint8_t lengh = 0;
	static uint32_t addr = 0;
	uint64_t *dta = NULL;
	static uint64_t *pdta = NULL;
	if(!address)	return HAL_ERROR;
	if(!lengh && !addr){
		lengh = sizeOfDataType;
		addr = address;
	}
	if(!pdta){
		dta = (uint64_t*)data;
	}else
		dta = pdta;
	*dta = *(__IO uint64_t*) addr;
	lengh -= 8;
	addr += 8;
	dta++;
	pdta = dta;
	if(lengh > 0 && lengh <= sizeOfDataType){
		return HAL_BUSY;
	}else
		return HAL_OK;
}
