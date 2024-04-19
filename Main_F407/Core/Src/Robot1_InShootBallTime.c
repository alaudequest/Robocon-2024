///*
// * Robot1_InShootBallTime.c
// *
// *  Created on: Apr 17, 2024
// *      Author: KHOA
// */
//
//#include "Robot1_InShootBallTime.h"
//#include "Robot1_Sensor.h"
//#include "ActuatorGun.h"
//#include "cmsis_os.h"
//bool inShootBallTime = false;
//_GamePad *_gamepad;
//bool isRowBallAbove = false;
//bool isDetectBallLeft = false;
//bool isDetectBallRight = false;
//Sensor_t collectBallLeft, collectBallRight;
//extern uint8_t Manual;
//extern int PlusControl;
//void ShootBallTime_Start(_GamePad *gamepad)
//{
//	inShootBallTime = true;
//	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
//	osDelay(100);
//	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
//	_gamepad = gamepad;
//	valve_ArmDown();
//	collectBallLeft = RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_LEFT);
//	collectBallRight= RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_RIGHT);
//	PlusControl = 2;// Điều khiển dạng chữ thập có thể quay góc
//	Manual = 1;
//}
//
//void ShootBallTime_Handle()
//{
//	if (!inShootBallTime || _gamepad == NULL)
//		return;
//	if (_gamepad->Up) {
//		osDelay(100);
//		if(_gamepad->Up){
//			// nếu ở hàng dưới banh thì đẩy xilanh trái mở sẵn sàng đón banh
//			isRowBallAbove = false;
//			valve_OpenLeftCollectBall();
//			valve_CloseRightCollectBall();
//		}
//
//	}
//	else if (_gamepad->Down) {
//		osDelay(100);
//		if(_gamepad->Down){
//			// nếu ở hàng trên thì kích xilanh phải
//			isRowBallAbove = true;
//			valve_OpenRightCollectBall();
//			valve_CloseLeftCollectBall();
//		}
//	}
//
//
//	if (_gamepad->Triangle && valve_IsProcessEnd()) {
//		osDelay(100);
//		if(_gamepad->Triangle){
//			// nếu ở hàng banh trên thì đưa banh vào
//			if (isRowBallAbove) {
//				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallRight);
//			}
//			else {
//				valve_ProcessBegin(ValveProcess_ShootBallTime_GetBallLeft);
//			}
//		}
//	}
//	if(_gamepad->Circle){
//		osDelay(100);
//		if(_gamepad->Circle){
//			RB1_EnableRuloShootBall();
//			RB1_CollectBallMotor_On();
//			if(isRowBallAbove){
//				RB1_SetTargetSpeedGun1(3000.0);
//				RB1_SetTargetSpeedGun2(3000.0);
//			}
//			else{
//				RB1_SetTargetSpeedGun1(4000.0);
//				RB1_SetTargetSpeedGun2(4000.0);
//			}
//		}
//	}
//	if (_gamepad->Square){
//		osDelay(100);
//		if(_gamepad->Square){
//			RB1_CollectBallMotor_Off();
//			RB1_SetTargetSpeedGun1(0);
//			RB1_SetTargetSpeedGun2(0);
//		}
//	}
//	if (_gamepad->Cross){
//		osDelay(1000.);
//		if(_gamepad->Cross){
//			ShootBallTime_Stop();
//			RB1_CollectBallMotor_Off();
//			RB1_SetTargetSpeedGun1(0);
//			RB1_SetTargetSpeedGun2(0);
//			PlusControl = 0;
//		}
//	}
//}
//
//void ShootBallTime_Stop()
//{
//	inShootBallTime = false;
//	valve_ArmUp();
//	valve_CloseLeftCollectBall();
//	valve_CloseRightCollectBall();
//	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
//	osDelay(100);
//	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
//	_gamepad = NULL;
//}
//
//bool IsInShootBallTime()
//{
//	return inShootBallTime;
//}
