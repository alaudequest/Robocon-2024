/*
 * Robot1_InShootBallTime.c
 *
 *  Created on: Apr 17, 2024
 *      Author: KHOA
 */

#include "Robot1_InShootBallTime.h"
#include "Robot1_Sensor.h"
#include "ActuatorGun.h"
#include "stdbool.h"
#include "cmsis_os.h"
bool inShootBallTime = false;
_GamePad *_gamepad;
bool isRowBallAbove = false;
bool isDetectBallLeft = false;
bool isDetectBallRight = false;
Sensor_t collectBallLeft, collectBallRight;
float aboveRowSpeed = 2500.0, belowRowSpeed = 3700.0;
extern uint8_t Manual;
extern int PlusControl;

bool isTriangleButtonPress = false;

static inline void ProcessDelay(uint32_t delayMs, uint8_t *step)
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
	PlusControl = 2;// Điều khiển dạng chữ thập có thể quay góc
	Manual = 1;
}

void RowBallAboveProcess()
{
	static uint8_t step = 0;
	float speedGun1 = RB1_GetSpeedRuloShootBall1();
	float speedGun2 = RB1_GetSpeedRuloShootBall2();

	if(   (speedGun1 > aboveRowSpeed - 300 && speedGun1 < aboveRowSpeed + 300)
		&&(speedGun2 > aboveRowSpeed - 300 && speedGun2 < aboveRowSpeed + 300) && step == 0){
		RB1_CollectBallMotor_On();
		step++;
	} else if(step == 1){
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		ProcessDelay(50, &step);
	}
	else if(step == 2){
		ProcessDelay(2500,&step);
	}
	else if(step == 3){
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		RB1_SetTargetSpeedGun1(0);
		RB1_SetTargetSpeedGun2(0);
		RB1_CollectBallMotor_Off();
		step = 0;
		isTriangleButtonPress = false;
	}
}

void BuzzerBeep(){
	static uint8_t step = 0;
	if(step == 1){
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		step++;
	}
	else if(step == 1){
		ProcessDelay(50,&step);
	}
	else if(step == 2){
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		step = 0;
	}
}


void ShootBallTime_Handle()
{
	if (!inShootBallTime || _gamepad == NULL)
		return;
	if (_gamepad->Up) {
		osDelay(100);
		if(_gamepad->Up){
			// nếu ở hàng dưới banh thì đẩy xilanh trái mở sẵn sàng đón banh
			isRowBallAbove = false;

			valve_OpenLeftCollectBall();
			valve_CloseRightCollectBall();
			valve_ArmDown();
		}

	}
	else if (_gamepad->Down) {
		osDelay(100);
		if(_gamepad->Down){
			// nếu ở hàng trên thì kích xilanh phải
			isRowBallAbove = true;
			valve_OpenRightCollectBall();
			valve_CloseLeftCollectBall();
			valve_ArmDown();
		}
	}


	if (_gamepad->Triangle && valve_IsProcessEnd()) {
		osDelay(100);
		if(_gamepad->Triangle){
			valve_ArmDown();
			// nếu ở hàng banh trên thì đưa banh vào
			if (isRowBallAbove) {
				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallRight);
//				RB1_SetTargetSpeedGun1(aboveRowSpeed);
//				RB1_SetTargetSpeedGun2(aboveRowSpeed);
//				isTriangleButtonPress = true;
			}
			else {
				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallLeft);
			}
		}
	}

//	if(isTriangleButtonPress){
//		RowBallAboveProcess();
//	}
	if(_gamepad->Circle){
		osDelay(100);
		if(_gamepad->Circle){
			RB1_CollectBallMotor_On();
			if(isRowBallAbove){
				RB1_SetTargetSpeedGun1(aboveRowSpeed);
				RB1_SetTargetSpeedGun2(aboveRowSpeed);
			}
			else{
				RB1_SetTargetSpeedGun1(belowRowSpeed);
				RB1_SetTargetSpeedGun2(belowRowSpeed);
			}
		}
	}
	if (_gamepad->Square){
		osDelay(100);
		if(_gamepad->Square){
			RB1_CollectBallMotor_Off();
			RB1_SetTargetSpeedGun1(0);
			RB1_SetTargetSpeedGun2(0);
		}
	}
	if (_gamepad->Cross){
		osDelay(1000);
		if(_gamepad->Cross){
			ShootBallTime_Stop();
			RB1_CollectBallMotor_Off();
			RB1_SetTargetSpeedGun1(0);
			RB1_SetTargetSpeedGun2(0);
			PlusControl = 0;
		}
	}

	// Nút giảm tốc
	if (_gamepad->R2){
		osDelay(100);
		if(_gamepad->R2){
			if(isRowBallAbove){
				aboveRowSpeed -=100;
				RB1_SetTargetSpeedGun1(aboveRowSpeed);
				RB1_SetTargetSpeedGun2(aboveRowSpeed);
			}
			else{
				belowRowSpeed -=100;
				RB1_SetTargetSpeedGun1(belowRowSpeed);
				RB1_SetTargetSpeedGun2(belowRowSpeed);
			}
		}

	}
	// Nút tăng tốc
	if (_gamepad->R1){
		osDelay(100);
		if(_gamepad->R1){
			if(isRowBallAbove){
				aboveRowSpeed +=100;
				RB1_SetTargetSpeedGun1(aboveRowSpeed);
				RB1_SetTargetSpeedGun2(aboveRowSpeed);
			}
			else{
				belowRowSpeed +=100;
				RB1_SetTargetSpeedGun1(belowRowSpeed);
				RB1_SetTargetSpeedGun2(belowRowSpeed);
			}
		}

	}

}

void ShootBallTime_Stop()
{
	inShootBallTime = false;
	valve_ArmUp();
	valve_CloseLeftCollectBall();
	valve_CloseRightCollectBall();
	RB1_CollectBallMotor_Off();
	RB1_SetTargetSpeedGun1(0);
	RB1_SetTargetSpeedGun2(0);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	osDelay(100);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	_gamepad = NULL;
}

bool IsInShootBallTime()
{
	return inShootBallTime;
}
