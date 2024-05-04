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
static float aboveRowSpeed = 2300.0, belowRowSpeed = 3200.0;
static uint8_t pwmAbove2 = 120, pwmAbove1 = 80, pwmBelow1 = 150, pwmBelow2 = 190;
extern uint8_t Manual;
extern int PlusControl;
static bool isOnDetectBallProcess = false;
#define SPEED_OFFSET 40
extern TIM_HandleTypeDef htim8;

void ShootBallTime_Start(_GamePad *gamepad)
{
	inShootBallTime = true;
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	osDelay(700);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	_gamepad = gamepad;
	valve_ArmDown();
	valve_HandHold();
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
			pwmAbove1 = 80;
			pwmAbove2 = pwmAbove1 + SPEED_OFFSET;
			valve_OpenLeftCollectBall();
			valve_CloseRightCollectBall();
			valve_ArmDown();
			valve_HandHold();
		}
	}
	else if (_gamepad->Down) {
		osDelay(100);
		if(_gamepad->Down){
			// nếu ở hàng trên thì kích xilanh phải
			isRowBallAbove = true;
			pwmBelow1 = 150;
			pwmBelow2 = pwmBelow1 + SPEED_OFFSET;
			valve_OpenRightCollectBall();
			valve_CloseLeftCollectBall();
			valve_ArmDown();
			valve_HandHold();
		}
	}

	if (_gamepad->Triangle && valve_IsProcessEnd()) {
		osDelay(100);
		if(_gamepad->Triangle){
			valve_ArmDown();
			valve_HandHold();
			// nếu ở hàng banh trên thì đưa banh vào
			if (isRowBallAbove) {
				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallRight);
//				RB1_SetTargetSpeedGun1(aboveRowSpeed);
//				RB1_SetTargetSpeedGun2(aboveRowSpeed);
			}
			else {
				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallLeft);
//				RB1_SetTargetSpeedGun1(belowRowSpeed);
//				RB1_SetTargetSpeedGun2(belowRowSpeed);
			}
		}
	}
//	if(valve_IsProcessEnd()){
//		RB1_SetTargetSpeedGun1(SHOOTBALL_IDLE_SPEED);
//		RB1_SetTargetSpeedGun2(SHOOTBALL_IDLE_SPEED);
//	}
	if(_gamepad->Circle){
		osDelay(100);
		if(_gamepad->Circle){
			RB1_CollectBallMotor_On();
			if(isRowBallAbove){
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pwmAbove2); // bắn 2
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,pwmAbove1);// bắn 1
//				RB1_SetTargetSpeedGun1(aboveRowSpeed);
//				RB1_SetTargetSpeedGun2(aboveRowSpeed);
			}
			else{
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pwmBelow2); // bắn 2
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,pwmBelow1);// bắn 1
//				RB1_SetTargetSpeedGun1(belowRowSpeed);
//				RB1_SetTargetSpeedGun2(belowRowSpeed);
			}
		}
	}
	if (_gamepad->Square){
		osDelay(100);
		if(_gamepad->Square){
			RB1_CollectBallMotor_Off();
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0); // bắn 2
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,0);// bắn 1
//			RB1_SetTargetSpeedGun1(0);
//			RB1_SetTargetSpeedGun2(0);
		}
	}
	if (_gamepad->Cross){
		osDelay(1000);
		if(_gamepad->Cross){
			ShootBallTime_Stop();
			RB1_CollectBallMotor_Off();
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0); // bắn 2
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,0);// bắn 1
//			RB1_SetTargetSpeedGun1(0);
//			RB1_SetTargetSpeedGun2(0);
			PlusControl = 0;
		}
	}

	// Nút giảm tốc
	if (_gamepad->L2){
		osDelay(100);
		if(_gamepad->L2 && BuzzerBeep_Start(1, 50, 0) == HAL_OK){
			if(isRowBallAbove){
				if(pwmAbove1 <= 80) {
					pwmAbove1 = 80;
					BuzzerBeep_Start(1, 500, 0);
				}
				else pwmAbove1 -= 10;
				pwmAbove2 = pwmAbove1 + SPEED_OFFSET;
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pwmAbove2); // bắn 2
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,pwmAbove1);// bắn 1
			}
			else{
				if(pwmBelow1 <= 150) {
					pwmBelow1 = 150;
					BuzzerBeep_Start(1, 500, 0);
				}
				else pwmBelow1 -= 10;
				pwmBelow2 = pwmBelow1 + SPEED_OFFSET;
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pwmBelow2); // bắn 2
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,pwmBelow1);// bắn 1
			}
		}
	}
	// Nút tăng tốc
	if (_gamepad->L1){
		osDelay(100);
		if(_gamepad->L1 && BuzzerBeep_Start(1, 50, 0) == HAL_OK){
			if(isRowBallAbove){
				if(pwmAbove1 >= 130) {
					pwmAbove1 = 130;
					BuzzerBeep_Start(1, 500, 0);
				}
				else pwmAbove1 += 10;
				pwmAbove2 = pwmAbove1 + SPEED_OFFSET;
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pwmAbove2); // bắn 2
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,pwmAbove1);// bắn 1
			}
			else{
				if(pwmBelow1 <= 200) {
					pwmBelow1 = 200;
					BuzzerBeep_Start(1, 500, 0);
				}
				else pwmBelow1 += 10;
				pwmBelow2 = pwmBelow1 + SPEED_OFFSET;
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pwmBelow2); // bắn 2
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,pwmBelow1);// bắn 1
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
	valve_HandRelease();
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
