/*
 * Robot1_InShootBallTime.c
 *
 *  Created on: Apr 17, 2024
 *      Author: KHOA
 */

#include "Robot1_InShootBallTime.h"
#include "cmsis_os.h"
bool inShootBallTime = false;
_GamePad *_gamepad;
bool isRowBallAbove = false;

void ShootBallTime_Start(_GamePad *gamepad)
{
	inShootBallTime = true;
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	osDelay(100);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	_gamepad = gamepad;
}

void ShootBallTime_Handle()
{
	if (!inShootBallTime || _gamepad == NULL)
		return;
	if (_gamepad->Up) {
		// nếu ở hàng banh trên thì đẩy xilanh trái mở sẵn sàng đón banh
		isRowBallAbove = true;
		valve_OpenLeftCollectBall();
	}
	else if (_gamepad->Down) {
		isRowBallAbove = false;
		valve_OpenRightCollectBall();
	}

	if (_gamepad->Circle && valve_IsProcessEnd()) {
		// nếu ở hàng banh trên thì đưa banh vào
		if (isRowBallAbove) {
			valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallLeft);
		}
		else {
			valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallRight);
		}
	}
	if (_gamepad->Cross)
		ShootBallTime_Stop();
}

void ShootBallTime_Stop()
{
	inShootBallTime = false;
	_gamepad = NULL;
}

bool IsInShootBallTime()
{
	return inShootBallTime;
}
