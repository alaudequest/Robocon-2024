/*
 * BoardParameter.c
 *
 *  Created on: Sep 23, 2023
 *      Author: SpiritBoi
 */
#include "BoardParameter.h"

uint8_t buzzerBeepRepeatTime = 0;
uint32_t buzzerOnDelayMs = 0;
uint32_t buzzerOffDelayMs = 0;
uint8_t buzzerStep = 0;

void ProcessDelay(uint32_t delayMs, uint8_t *step)
{
	static bool isOnDelay = false;
	static uint32_t DelayTick = 0;
	if (isOnDelay != true) {
		DelayTick = HAL_GetTick();
		isOnDelay = true;
	}
	if (HAL_GetTick() - DelayTick >= delayMs && isOnDelay) {
		isOnDelay = false;
		*step = *step + 1;
	}
}


HAL_StatusTypeDef BuzzerBeep_Start(uint8_t repeatTime, uint32_t onDelayMs,uint32_t offDelayMs)
{
	if(buzzerStep != 0) return HAL_ERROR;
	buzzerBeepRepeatTime = repeatTime - 1;
	buzzerOnDelayMs = onDelayMs;
	buzzerOffDelayMs = offDelayMs;
	buzzerStep = 1;
	return HAL_OK;
}

void BuzzerBeepProcess()
{
	switch(buzzerStep){
	case 1:
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		buzzerStep++;
		break;
	case 2:
		ProcessDelay(buzzerOnDelayMs,&buzzerStep);
		break;
	case 3:
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		buzzerStep++;
		break;
	case 4:
		ProcessDelay(buzzerOffDelayMs,&buzzerStep);
		break;
	case 5:
		if(buzzerBeepRepeatTime > 0) {
			buzzerBeepRepeatTime --;
			buzzerStep= 1;
		}
		else
			buzzerStep= 0;
		break;
	}
}

