/*
 * RobotFlashData.h
 *
 *  Created on: Apr 28, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_ROBOTFLASHDATA_H_
#define INC_ROBOTFLASHDATA_H_

#include "main.h"
#include "BoardParameter.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"



void RBFlash_EndOperationHandler(uint32_t ReturnValue);
void RBFlash_ErrorHandler(uint32_t ReturnValue);
void RBFlash_ReadDataFromFlash(void *pData, uint8_t sizeOfData, uint32_t address);
void RBFlash_WriteToFlash(void *pData, uint8_t sizeOfData, uint32_t address);
void RBFlash_ErrorHandler(uint32_t ReturnValue);
void RBFlash_Test();
#endif /* INC_ROBOTFLASHDATA_H_ */

