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
static TeamColor teamColor = 0;
static bool _dataChanged = false;

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

HAL_StatusTypeDef BuzzerBeep_Start(uint8_t repeatTime, uint32_t onDelayMs, uint32_t offDelayMs)
{
	if (buzzerStep != 0)
		return HAL_ERROR;
	buzzerBeepRepeatTime = repeatTime - 1;
	buzzerOnDelayMs = onDelayMs;
	buzzerOffDelayMs = offDelayMs;
	buzzerStep = 1;
	return HAL_OK;
}

void BuzzerBeepProcess()
{
	switch (buzzerStep) {
		case 1:
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
			buzzerStep++;
			break;
		case 2:
			ProcessDelay(buzzerOnDelayMs, &buzzerStep);
			break;
		case 3:
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
			buzzerStep++;
			break;
		case 4:
			ProcessDelay(buzzerOffDelayMs, &buzzerStep);
			break;
		case 5:
			if (buzzerBeepRepeatTime > 0) {
				buzzerBeepRepeatTime--;
				buzzerStep = 1;
			}
			else
				buzzerStep = 0;
			break;
	}
}

void DetectSignalButtonProcess(SignalButtonColor *color)
{
	static uint8_t step = 0;
	GPIO_TypeDef *gpio;
	uint16_t gpioPin;
	switch (*color) {
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
	if (*color != 0 && step == 0)
		step = 1;
	else if (*color == 0)
		step = 0;
	switch (step) {
		case 1:
			ProcessDelay(300, &step);
			break;
		case 2:
			if (!HAL_GPIO_ReadPin(gpio, gpioPin)) {
				if (BuzzerBeep_Start(1, 50, 0) == HAL_OK) {
					if (_pSignalBtnPressed != NULL)
						_pSignalBtnPressed(*color);
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
	if (!HAL_GPIO_ReadPin(RobotSignalBtn_RED_GPIO_Port, RobotSignalBtn_RED_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_RED;
	else if (!HAL_GPIO_ReadPin(RobotSignalBtn_YELLOW_GPIO_Port, RobotSignalBtn_YELLOW_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_YELLOW;
	else if (!HAL_GPIO_ReadPin(RobotSignalBtn_BLUE_GPIO_Port, RobotSignalBtn_BLUE_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_BLUE;
	else if (!HAL_GPIO_ReadPin(RobotSignalBtn_GREEN_GPIO_Port, RobotSignalBtn_GREEN_Pin) && currentPressedButton == 0)
		currentPressedButton = SIGBTN_GREEN;
	DetectSignalButtonProcess(&currentPressedButton);
}



#if defined(BOARD_MAINF4_ROBOT1)

#elif defined(BOARD_MAINF4_ROBOT2)


Robot2PlayGroundData robotPlayGroundData;

void BrdParam_SetCustomGamepad(CustomGamepad_t gamepad)
{
	// chỉ cần có một sự thay đổi bất kỳ giữa dữ liệu mới và dữ liệu board thì gán luôn không cần kiểm tra tiếp
	if (robotPlayGroundData.gp.siloNum != gamepad.siloNum) {
		robotPlayGroundData.gp = gamepad;
		_dataChanged = true;
		return;
	}
	for (uint8_t i = 0; i < 6; i++) {
		if (robotPlayGroundData.gp.ballMatrix[i] != gamepad.ballMatrix[i]) {
			// chỉ cần có một sự thay đổi bất kỳ giữa dữ liệu mới và dữ liệu board thì gán luôn không cần kiểm tra tiếp
			robotPlayGroundData.gp = gamepad;
			_dataChanged = true;
			return;
		}
	}
}

CustomGamepad_t BrdParam_GetCustomGamepad()
{
	return robotPlayGroundData.gp;
}

void BrdParam_SetTeamColor(TeamColor color)
{
	if (robotPlayGroundData.teamColor != color) {
		robotPlayGroundData.teamColor = color;
		_dataChanged = true;
	}
}

TeamColor BrdParam_GetTeamColor()
{
	return robotPlayGroundData.teamColor;
}

void BrdParam_SetBallSuccess(uint8_t ballGetSuccess)
{
	if (robotPlayGroundData.ballCollectSuccess != ballGetSuccess) {
		robotPlayGroundData.ballCollectSuccess = ballGetSuccess;
		_dataChanged = true;
	}
}

uint8_t BrdParam_GetBallSuccess()
{
	return robotPlayGroundData.ballCollectSuccess;
}

void BrdParam_SetBallInSilo(uint8_t ball, uint8_t silo){
	robotPlayGroundData.ballInSilo[silo] = ball;
}

uint8_t BrdParam_GetBallInSilo(uint8_t silo){
	return robotPlayGroundData.ballInSilo[silo];
}

void BrdParam_GetDataFromFlash()
{
	RBFlash_ReadDataFromFlash((void*)&robotPlayGroundData, sizeof(robotPlayGroundData), FLASH_ROBOT_PLAYGROUND_DATA_ADDRESS);
}

void BrdParam_SaveDataToFlash()
{
	if (_dataChanged) {
		RBFlash_WriteToFlash((void*)&robotPlayGroundData, sizeof(robotPlayGroundData), FLASH_ROBOT_PLAYGROUND_DATA_ADDRESS);
		_dataChanged = false;
	}
}
#else
#error "Not define board selected in main.h"
#endif
