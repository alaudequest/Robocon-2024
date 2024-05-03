/*
 * BoardParameter.c
 *
 *  Created on: Sep 23, 2023
 *      Author: SpiritBoi
 */
#include "BoardParameter.h"

static uint8_t buzzerBeepRepeatTime = 0;
static uint32_t buzzerOnDelayMs = 0;
static uint32_t buzzerOffDelayMs = 0;
static uint8_t buzzerStep = 0;
static SignalButtonColor currentPressedButton = 0;
pSignalBtnPressed _pSignalBtnPressed;
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

void DetectSignalButtonProcess(SignalButtonColor *color)
{
	static uint8_t step = 0;
	GPIO_TypeDef *gpio;
	uint16_t gpioPin;
	switch(*color){
	case SIGBTN_RED:
		gpio = RobotSignalBtn_RED_GPIO_Port;
		gpioPin = RobotSignalBtn_RED_Pin;
		break;
	case SIGBTN_YELLOW:
		gpio = RobotSignalBtn_YELLOW_GPIO_Port;
		gpioPin = RobotSignalBtn_YELLOW_Pin;
		break;
	case SIGBTN_BLUE:
		gpio = RobotSignalBtn_BLUE_GPIO_Port;
		gpioPin = RobotSignalBtn_BLUE_Pin;
		break;
	case SIGBTN_GREEN:
		gpio = RobotSignalBtn_GREEN_GPIO_Port;
		gpioPin = RobotSignalBtn_GREEN_Pin;
		break;
	}
	if(*color!=0 && step == 0)
		step = 1;
	else if(*color==0)
		step = 0;
	switch(step){
	case 1:
		ProcessDelay(300, &step);
		break;
	case 2:
		if(!HAL_GPIO_ReadPin(gpio, gpioPin)){
			if(BuzzerBeep_Start(1, 50, 0) == HAL_OK){
				if(_pSignalBtnPressed != NULL)_pSignalBtnPressed(*color);
			}
		}
		step++;
		break;
	case 3:
		step = 0;
		*color = 0;
		break;
	}
}

void RobotSignalButton_RegisterButtonPressedCallback(void (*pSignalBtnPressed)(SignalButtonColor))
{
	_pSignalBtnPressed = pSignalBtnPressed;
}

void RobotSignalButton_ScanButton()
{
	HAL_GPIO_WritePin(RobotSignalBtn_VCC_GPIO_Port, RobotSignalBtn_VCC_Pin, 1);
	HAL_GPIO_WritePin(RobotSignalBtn_GND_GPIO_Port, RobotSignalBtn_GND_Pin, 0);
	if(!HAL_GPIO_ReadPin(RobotSignalBtn_RED_GPIO_Port, RobotSignalBtn_RED_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_RED;
	else if(!HAL_GPIO_ReadPin(RobotSignalBtn_YELLOW_GPIO_Port, RobotSignalBtn_YELLOW_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_YELLOW;
	else if(!HAL_GPIO_ReadPin(RobotSignalBtn_BLUE_GPIO_Port, RobotSignalBtn_BLUE_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_BLUE;
	else if(!HAL_GPIO_ReadPin(RobotSignalBtn_GREEN_GPIO_Port, RobotSignalBtn_GREEN_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_GREEN;
	DetectSignalButtonProcess(&currentPressedButton);
}

