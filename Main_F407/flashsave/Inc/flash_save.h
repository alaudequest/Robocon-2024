/*
 * flash_save.h
 *
 *  Created on: Sep 29, 2023
 *      Author: NamDHay
 */

#ifndef INC_FLASH_SAVE_H_
#define INC_FLASH_SAVE_H_

#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "BoardParameter.h"
#include "stdbool.h"

void Flash_Erase(uint32_t addr);

HAL_StatusTypeDef Flash_Write(uint32_t address, void *data, size_t sizeofDataType);
HAL_StatusTypeDef Flash_Read(uint32_t address, void *data, size_t sizeOfDataType);

#endif /* INC_FLASH_SAVE_H_ */
