/*
 * Robot1_InShootBallTime.c
 *
 *  Created on: Apr 17, 2024
 *      Author: KHOA
 */

#include "Robot1_InShootBallTime.h"
#include "BoardParameter.h"
#include "Robot1_Sensor.h"
#include "ActuatorGun.h"
#include "stdbool.h"
#include "cmsis_os.h"
static bool inShootBallTime = false;
static _GamePad *_gamepad;
static bool isRowBallAbove = false;
static Sensor_t collectBallLeft, collectBallRight;
static Sensor_t *currentDetectSensor = NULL;
static float aboveRowSpeed = 2800.0, belowRowSpeed = 3700.0;
extern uint8_t Manual;
extern int PlusControl;
static bool isOnDetectBallProcess = false;


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


static void DetectBallProcess(Sensor_t *sensor, bool *isOnDetectBallProcess){
	static uint8_t step = 0;
	if(sensor == NULL) return;
	if(*isOnDetectBallProcess == true && step == 0){
		step = 1;
	} else if(*isOnDetectBallProcess == false){
		step = 0;
	}
	switch(step){
	case 1:
		// sau khi cảm biến bắt được bóng thì delay 100ms
		ProcessDelay(100, &step);
		break;
	case 2:
		// đọc lại cảm biến để chắc chắn là không nhiễu, sau đó cho buzzer kêu, kết thúc quá trình đọc
		if(HAL_GPIO_ReadPin(sensor->sensorPort, sensor->sensorPin)){
			if(BuzzerBeep_Start(1, 50, 0) == HAL_OK)
				step++;
		}
		break;
	case 3:
		ProcessDelay(1000, &step);
		break;
	case 4:
		// nếu cảm biến hết phát hiện bóng thì reset quá trình về 0
		if(!HAL_GPIO_ReadPin(sensor->sensorPort, sensor->sensorPin))
			step++;
		else if(HAL_GPIO_ReadPin(sensor->sensorPort, sensor->sensorPin)){
			step = 1;
		}
		break;
	case 5:
		ProcessDelay(100, &step);
		break;
	case 6:
		if(!HAL_GPIO_ReadPin(sensor->sensorPort, sensor->sensorPin))
			step++;
		break;
	case 7:
		step = 0;
		sensor = NULL;
		*isOnDetectBallProcess = false;
		break;
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
			}
			else {
				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallLeft);
			}
		}
	}
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
		if(_gamepad->R2 && BuzzerBeep_Start(1, 50, 0) == HAL_OK){
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
		if(_gamepad->R1 && BuzzerBeep_Start(1, 50, 0) == HAL_OK){
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
	if(isRowBallAbove){
		if(HAL_GPIO_ReadPin(collectBallRight.sensorPort, collectBallRight.sensorPin)){
			isOnDetectBallProcess = true;
			currentDetectSensor = &collectBallRight;
		}
	}
	else {
		if(HAL_GPIO_ReadPin(collectBallLeft.sensorPort, collectBallLeft.sensorPin)){
			isOnDetectBallProcess = true;
			currentDetectSensor = &collectBallLeft;
		}
	}
	DetectBallProcess(currentDetectSensor, &isOnDetectBallProcess);
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
