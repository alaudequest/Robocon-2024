/*
 * ActuatorValve.c
 *
 *  Created on: Jan 15, 2024
 *      Author: SpiritBoi
 */

#include <RB1ActuatorValve.h>
#include "stdbool.h"
#include "cmsis_os.h"
static HC595 valve;
static uint8_t valveProcessStep = 0;
static ValveProcessName valveProcessName = 0;

static inline void ProcessEnd();
static inline void ProcessDelay(uint32_t delayMs);

void valve_ProcessBegin(ValveProcessName _valveProcessName)
{
	if (valveProcessStep != 0)
		return;
	valveProcessName = _valveProcessName;
	valveProcessStep = 1;
}

void valve_Init() {
	HC595_AssignPin(&valve, HC595_CLK_GPIO_Port, HC595_CLK_Pin, HC595_CLK);
	HC595_AssignPin(&valve, HC595_RCLK_GPIO_Port, HC595_RCLK_Pin, HC595_LATCH);
	HC595_AssignPin(&valve, HC595_DATA_GPIO_Port, HC595_DATA_Pin, HC595_DS);
	HC595_AssignPin(&valve, HC595_OE_GPIO_Port, HC595_OE_Pin, HC595_OE);
	HC595_ClearByteOutput(0xff);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_OutputAllPin(uint8_t valveArrayOutput)
{
	HC595_ClearByteOutput(0xff);
	HC595_SetByteOutput(valveArrayOutput);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_Output(uint8_t outputPort, bool on) {
	if (on)
		HC595_SetBitOutput(outputPort);
	else
		HC595_ClearBitOutput(outputPort);
	HC595_ShiftOut(NULL, 1, 1);
}

void ProcessCatchAndHold()
{
	if (valveProcessName != ValveProcess_CatchAndHold)
		return;
	switch (valveProcessStep) {
		case 1:
			valve_ArmDown();
			valveProcessStep++;
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			break;
		case 2:
			ProcessDelay(600);
			break;
		case 3:
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 0);
			valve_HandHold();
			valveProcessStep++;
			break;
		case 4:
			ProcessDelay(100);
			break;
		case 5:
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			valve_ArmUp();
			valveProcessStep++;
			break;
		case 6:
			ProcessEnd();
			break;
	}
}

static void InShootBallTime_ProcessStart()
{
	if (valveProcessName != ValveProcess_ShootBallTime_Start)
		return;
	switch (valveProcessStep) {
		case 1:
			valve_ArmDown();
			valveProcessStep++;
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			break;
		case 2:
			ProcessDelay(1000);
			break;
		case 3:
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 0);
			valve_HandRelease();
			valveProcessStep++;
			break;
		case 4:
			ProcessEnd();
			break;
	}

}

static void InShootBallTime_ProcessStop()
{
	if (valveProcessName != ValveProcess_ShootBallTime_Stop)
		return;
	switch (valveProcessStep) {
		case 1:
			valve_ArmUp();
			valveProcessStep++;
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			break;
		case 2:
			ProcessDelay(1000);
			break;
		case 3:
			HC595_ClearBitOutput(VALVE_HAND_LEFT_HC595_PIN); // mở tay gắp trái
			HC595_ClearBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // mở tay gắp phải
			HC595_ClearBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
			HC595_ClearBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
			valveProcessStep++;
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 0);
			break;
		case 4:
			ProcessEnd();
			break;
	}
}

static void InShootBallTime_ProcessGetBallRight()
{
	if (valveProcessName != ValveProcess_ShootBallTime_GetBallRight)
		return;
	switch (valveProcessStep) {
		case 1:
			valve_CloseRightCollectBall();
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			valveProcessStep++;
			break;
		case 2:
			ProcessDelay(1000);
			break;
		case 3:
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 0);
			valve_OpenRightCollectBall();
			valveProcessStep++;
			break;
		case 4:
			ProcessEnd();
			break;
	}
}

static void InShootBallTime_ProcessGetBallLeft()
{
	if (valveProcessName != ValveProcess_ShootBallTime_GetBallLeft)
		return;
	switch (valveProcessStep) {
		case 1:
			valve_CloseLeftCollectBall();
			valveProcessStep++;
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			break;
		case 2:
			ProcessDelay(1000);
			break;
		case 3:
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 0);
			valve_OpenLeftCollectBall();
			valveProcessStep++;
			break;
		case 4:
			ProcessEnd();
			break;
	}
}

static void InShootBallTime_ProcessReset()
{
	if (valveProcessName != ValveProcess_ShootBallTime_Reset)
		return;
	switch (valveProcessStep) {
		case 1:
			valve_ArmDown();
			valveProcessStep++;
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			break;
		case 2:
			ProcessDelay(500);
			break;
		case 3:
			valve_HandRelease();
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 0);
			valveProcessStep++;
			break;
		case 4:
			ProcessDelay(500);
			break;
		case 5:
			HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
			valve_ArmUp();
			valveProcessStep++;
			break;
		case 6:
			ProcessEnd();
			break;
	}
}

void RB1_valve_ProcessManager()
{
	ProcessCatchAndHold();
	InShootBallTime_ProcessStart();
	InShootBallTime_ProcessStop();
	InShootBallTime_ProcessGetBallRight();
	InShootBallTime_ProcessGetBallLeft();
	InShootBallTime_ProcessReset();
}

void valve_ArmUp()
{
	// nâng cánh tay trái và phải
	HC595_ClearBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_ClearBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_ArmDown()
{
	// hạ cánh tay trái và phải
	HC595_SetBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_SetBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_HandHold()
{
	HC595_SetBitOutput(VALVE_HAND_LEFT_HC595_PIN); // kẹp tay gắp trái
	HC595_SetBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // kẹp tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_HandRelease()
{
	HC595_ClearBitOutput(VALVE_HAND_LEFT_HC595_PIN); // mở tay gắp trái
	HC595_ClearBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // mở tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_OpenLeftCollectBall()
{
	// đẩy cơ cấu lùa banh ra, chờ bắt banh
	HC595_SetBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_CloseLeftCollectBall()
{
	// thu cơ cấu lùa banh vào
	HC595_ClearBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_CloseRightCollectBall()
{
	// thu cơ cấu lùa banh vào
	HC595_ClearBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_OpenRightCollectBall()
{
	// đẩy cơ cấu lùa banh ra, chờ bắt banh
	HC595_SetBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void ProcessEnd()
{
	valveProcessStep = 0;
}

static inline void ProcessDelay(uint32_t delayMs)
{
	static bool isOnDelay = false;
	static uint32_t valveDelayTick = 0;
	if (isOnDelay != true) {
		valveDelayTick = HAL_GetTick();
		isOnDelay = true;
	}
	if (HAL_GetTick() - valveDelayTick >= delayMs && isOnDelay) {
		isOnDelay = false;
		valveProcessStep++;
	}
}

void valve_TestPin(pinName pin) {
	HC595_TestPin(pin);
}

bool valve_IsProcessEnd() {
	return !valveProcessStep;
}
