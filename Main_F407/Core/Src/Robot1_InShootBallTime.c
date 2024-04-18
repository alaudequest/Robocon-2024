/*
 * Robot1_InShootBallTime.c
 *
 *  Created on: Apr 17, 2024
 *      Author: KHOA
 */

#include "Robot1_InShootBallTime.h"
#include "Robot1_Sensor.h"
#include "cmsis_os.h"
bool inShootBallTime = false;
_GamePad *_gamepad;
bool isRowBallAbove = false;
bool isDetectBallLeft = false;
bool isDetectBallRight = false;
Sensor_t collectBallLeft, collectBallRight;

void ShootBallTime_Start(_GamePad *gamepad)
{
	inShootBallTime = true;
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	osDelay(100);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	_gamepad = gamepad;
	valve_ArmDown();
	collectBallLeft = RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_LEFT);
	collectBallRight= RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_RIGHT);
}

void ShootBallTime_Handle()
{
	if (!inShootBallTime || _gamepad == NULL)
		return;
	if (_gamepad->Up) {
		// nếu ở hàng banh trên thì đẩy xilanh trái mở sẵn sàng đón banh
		isRowBallAbove = true;
		isDetectBallRight = false;
		valve_OpenLeftCollectBall();
		valve_CloseRightCollectBall();
		if(HAL_GPIO_ReadPin(collectBallLeft.sensorPort, collectBallLeft.sensorPin) && !isDetectBallLeft){
			isDetectBallLeft = true;
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
			osDelay(100);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		}
		else if(!HAL_GPIO_ReadPin(collectBallLeft.sensorPort, collectBallLeft.sensorPin) && isDetectBallLeft){
			isDetectBallLeft = false;
		}
	}
	else if (_gamepad->Down) {
		isDetectBallLeft = false;
		isRowBallAbove = false;
		valve_OpenRightCollectBall();
		valve_CloseLeftCollectBall();
		if(HAL_GPIO_ReadPin(collectBallRight.sensorPort, collectBallRight.sensorPin) && !isDetectBallRight){
			isDetectBallRight = true;
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
			osDelay(100);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		}
		else if(!HAL_GPIO_ReadPin(collectBallRight.sensorPort, collectBallRight.sensorPin) && isDetectBallRight){
			isDetectBallRight = false;
		}
	}


	if (_gamepad->Triangle && valve_IsProcessEnd()) {
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
	valve_ArmUp();
	valve_CloseLeftCollectBall();
	valve_CloseRightCollectBall();
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	osDelay(100);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	_gamepad = NULL;
}

bool IsInShootBallTime()
{
	return inShootBallTime;
}
