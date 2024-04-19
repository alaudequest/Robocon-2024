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
static uint32_t valveDelayTick = 0;
static uint8_t valveProcessStep = 0;

static inline void ProcessArmUp(uint8_t expectedStep);
static inline void ProcessArmDown(uint8_t expectedStep);
static inline void ProcessHandHold(uint8_t expectedStep);
static inline void ProcessHandRelease(uint8_t expectedStep);
static inline void ProcessBegin();
static inline void ProcessEnd(uint8_t expectedStep);
static inline void ProcessDelay(uint8_t expectedStep, uint32_t delayMs);
static inline void ArmUp();
static inline void ArmDown();
static inline void HandHold();
static inline void HandRelease();

void valve_ProcessBegin()
{
	ProcessBegin();
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

void valve_BothCatch()
{

	ProcessArmDown(1);
	ProcessDelay(2, 500);
	ProcessHandHold(3);
	ProcessDelay(4, 500);
	ProcessArmUp(5);
	ProcessDelay(6, 500);
	ProcessEnd(7);
}

void valve_BothRelease()
{
	ProcessBegin();
	ProcessArmDown(1);
	ProcessDelay(2, 500);
	ProcessHandRelease(3);
	ProcessDelay(4, 500);
	ProcessArmUp(5);
	ProcessDelay(6, 500);
	ProcessEnd(7);
}

void valve_Reset()
{
	ProcessBegin();
	ProcessArmDown(1);
	ProcessDelay(2, 500);
	ProcessHandRelease(3);
	ProcessDelay(4, 500);
	ProcessArmUp(5);
	ProcessEnd(6);
}

void valve_ArmUp()
{
	ArmDown();
}

void valve_ArmDown()
{
	ArmDown();
}

void valve_HandHold()
{
	HandHold();
}

void valve_HandRelease()
{
	HandRelease();
}

void CloseLeftCollectBall()
{
	// thu cơ cấu lùa banh vào
	HC595_ClearBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void OpenLeftCollectBall()
{

	// đẩy cơ cấu lùa banh ra, chờ bắt banh
	HC595_SetBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void CloseRightCollectBall()
{
	// thu cơ cấu lùa banh vào
	HC595_ClearBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void OpenRightCollectBall()
{
	// đẩy cơ cấu lùa banh ra, chờ bắt banh
	HC595_SetBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_TestPin(pinName pin) {
	HC595_TestPin(pin);
}



static inline void ArmUp()
{
	// nâng cánh tay trái và phải

	HC595_ClearBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_ClearBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);


}

static inline void ArmDown()
{
	// hạ cánh tay trái và phải
	HC595_SetBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_SetBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void HandHold()
{
	HC595_SetBitOutput(VALVE_HAND_LEFT_HC595_PIN); // kẹp tay gắp trái
	HC595_SetBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // kẹp tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void HandRelease()
{
	HC595_ClearBitOutput(VALVE_HAND_LEFT_HC595_PIN); // mở tay gắp trái
	HC595_ClearBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // mở tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void ProcessOpenLeftCollectBall(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	OpenLeftCollectBall();
	valveProcessStep++;
}

static inline void ProcessOpenRightCollectBall(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	OpenRightCollectBall();
	valveProcessStep++;
}

static inline void ProcessCloseLeftCollectBall(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	CloseLeftCollectBall();
	valveProcessStep++;
}

static inline void ProcessCloseRightCollectBall(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	CloseRightCollectBall();
	valveProcessStep++;
}

static inline void ProcessDelay(uint8_t expectedStep, uint32_t delayMs)
{
	static bool isOnDelay = false;
	if (expectedStep != valveProcessStep)
		return;
	if (isOnDelay != true) {
		valveDelayTick = HAL_GetTick();
		isOnDelay = true;
	}
	if (HAL_GetTick() - valveDelayTick >= delayMs) {
		isOnDelay = false;
		valveProcessStep++;
	}
}

static inline void ProcessArmUp(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	ArmUp();
	valveProcessStep++;
}

static inline void ProcessArmDown(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	ArmDown();
	valveProcessStep++;
}

static inline void ProcessHandHold(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	HandHold();
	valveProcessStep++;
}

static inline void ProcessHandRelease(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
		return;
	HandRelease();
	valveProcessStep++;
}

static inline void ProcessBegin()
{
	valveProcessStep = 1;
}

static inline void ProcessEnd(uint8_t expectedStep)
{
	if (expectedStep != valveProcessStep)
			return;
	valveProcessStep = 0;
}

void valve_InShootBallTime_Start()
{
	ProcessArmDown(1);
	ProcessDelay(2, 200);
	ProcessHandRelease(3);
	ProcessDelay(4, 500);
	ProcessEnd(5);
}

void valve_InShootBallTime_Stop()
{

	ProcessArmUp(1);
	ProcessDelay(2, 200);
	ProcessHandRelease(3);
	ProcessCloseLeftCollectBall(5);
	ProcessCloseRightCollectBall(6);
	ProcessEnd(7);
}

void valve_InShootBallTime_ReadyLeftCollectBall()
{
	ProcessOpenLeftCollectBall(1);
	ProcessDelay(2, 1000);
	ProcessEnd(3);
}

void valve_InShootBallTime_ReadyRightCollectBall()
{
	ProcessOpenRightCollectBall(1);
	ProcessDelay(2, 1000);
	ProcessEnd(3);
}

void valve_InShootBallTime_GetBallRight()
{
	ProcessCloseRightCollectBall(1);
	ProcessDelay(2, 1000);
	ProcessOpenRightCollectBall(3);
	ProcessDelay(4, 1000);
	ProcessEnd(5);
}

void valve_InShootBallTime_GetBallLeft()
{
	ProcessCloseLeftCollectBall(1);
	ProcessDelay(2, 1000);
	ProcessOpenLeftCollectBall(3);
	ProcessDelay(4, 1000);
	ProcessEnd(5);
}

bool valve_IsProcessEnd() {
	return !valveProcessStep;
}
