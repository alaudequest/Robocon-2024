/*
 * ProcessDelay.c
 *
 *  Created on: Mar 21, 2024
 *      Author: KHOA
 */

#include "ProcessDelay.h"
#define MAX_DELAY_STAGE 3
DelayStage delayStage = 0;
DelayParameter_t arrDelay[MAX_DELAY_STAGE] = {0};

uint16_t currentDelay = 0;
void processDelay_PutDelayValueToArray(uint32_t milisecond, uint8_t stage) {
	if(stage > MAX_DELAY_STAGE) return;
	arrDelay[stage].delay = milisecond;
}

uint32_t processDelay_GetDelayValueFromArray(uint8_t stage) {
	return arrDelay[stage].delay;
}

/**
 * Must be place in HAL_TIM_PeriodElapsedCallback
 */
void processDelay_DelayInISR() {
	if(currentDelay < arrDelay[delayStage].delay)
		currentDelay++;
	else if(delayStage < MAX_DELAY_STAGE) {
		currentDelay = 0;
		delayStage++;
	}

}


