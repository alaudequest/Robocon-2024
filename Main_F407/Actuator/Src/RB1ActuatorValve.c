/*
 * ActuatorValve.c
 *
 *  Created on: Jan 15, 2024
 *      Author: SpiritBoi
 */

#include <RB1ActuatorValve.h>
#include "stdbool.h"
#include "cmsis_os.h"
HC595 valve;
uint32_t valveDelayTick = 0;
uint8_t valveProcessStep = 0;

static inline void ProcessArmUp(uint8_t expectedStep);
static inline void ProcessArmDown(uint8_t expectedStep);
static inline void ProcessHandHold(uint8_t expectedStep);
static inline void ProcessHandRelease(uint8_t expectedStep);
static inline void ProcessBegin();
static inline void ProcessEnd();
static inline void ProcessDelay(uint8_t expectedStep, uint32_t delayMs);

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

void valve_TestBlinkAll() {
	HC595_SetByteOutput(0xff);
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(2000);
	HC595_ClearByteOutput(0xff);
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(2000);
}

void valve_BothCatch()
{
	ProcessBegin();
	ProcessArmDown(1);
	ProcessDelay(2, 500);
	ProcessHandHold(3);
	ProcessDelay(4, 500);
	ProcessArmUp(5);
	ProcessDelay(6, 500);
	ProcessEnd();
}

void valve_BothRelease()
{
	ProcessBegin();
	ProcessArmDown(1);
	ProcessDelay(2, 500);
	ProcessHandRelease(4);
	ProcessDelay(5, 500);
	ProcessArmUp(6);
	ProcessEnd();
}

void valve_LeftCollectBall() {
	// thu cơ cấu lùa banh vào
	HC595_ClearBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_LeftWaitCollectBall() {

	// đẩy cơ cấu lùa banh ra, chờ bắt banh
	HC595_SetBitOutput(VALVE_COLLECT_BALL_LEFT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_RightCollectBall() {
	// thu cơ cấu lùa banh vào
	HC595_ClearBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_RightWaitCollectBall() {
	// đẩy cơ cấu lùa banh ra, chờ bắt banh
	HC595_SetBitOutput(VALVE_COLLECT_BALL_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_TestPin(pinName pin) {
	HC595_TestPin(pin);
}

static inline void ArmUp() {
	// nâng cánh tay trái và phải
	HC595_SetBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_SetBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void ArmDown() {
	// hạ cánh tay trái và phải
	HC595_ClearBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_ClearBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void HandHold() {
	HC595_SetBitOutput(VALVE_HAND_LEFT_HC595_PIN); // kẹp tay gắp trái
	HC595_SetBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // kẹp tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void HandRelease() {
	HC595_ClearBitOutput(VALVE_HAND_LEFT_HC595_PIN); // mở tay gắp trái
	HC595_ClearBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // mở tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
}

static inline void ProcessDelay(uint8_t expectedStep, uint32_t delayMs)
{
	static bool isOnDelay = false;
	if (expectedStep == false)
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
	if (expectedStep == false)
		return;
	ArmUp();
	valveProcessStep++;
}

static inline void ProcessArmDown(uint8_t expectedStep)
{
	if (expectedStep == false)
		return;
	ArmDown();
	valveProcessStep++;
}

static inline void ProcessHandHold(uint8_t expectedStep)
{
	if (expectedStep == false)
		return;
	HandHold();
	valveProcessStep++;
}

static inline void ProcessHandRelease(uint8_t expectedStep)
{
	if (expectedStep == false)
		return;
	HandRelease();
	valveProcessStep++;
}

static inline void ProcessBegin()
{
	valveProcessStep = 1;
}

static inline void ProcessEnd()
{
	valveProcessStep = 0;
}
