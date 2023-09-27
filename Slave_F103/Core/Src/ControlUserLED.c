/*
 * ControlUserLED.c
 *
 *  Created on: Sep 27, 2023
 *      Author: KHOA
 */
#include "ControlUserLED.h"

UserLED_t blueLED;

void usrled_RunLED()
{
	switch(blueLED.mode){
	case LED_BLINK:
		uint16_t delay = blueLED.delay;
		static uint32_t tick = 0;
		static uint16_t internalDelay = 100;
		if(delay) internalDelay = delay;
		if(HAL_GetTick() - tick > internalDelay){
			tick = HAL_GetTick();
			HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
		}
		break;
	case LED_FADE:
		break;
	case LED_STATE:
		break;
	}
}

void usrled_SetModeLED(LED_Mode mode){blueLED.mode = mode;}
LED_Mode usrled_GetModeLED(){return blueLED.mode;}
