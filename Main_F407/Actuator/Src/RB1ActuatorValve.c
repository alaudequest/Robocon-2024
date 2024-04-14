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

void valve_Init() {
	HC595_AssignPin(&valve, HC595_CLK_GPIO_Port, HC595_CLK_Pin, HC595_CLK);
	HC595_AssignPin(&valve, HC595_RCLK_GPIO_Port, HC595_RCLK_Pin, HC595_LATCH);
	HC595_AssignPin(&valve, HC595_DATA_GPIO_Port, HC595_DATA_Pin, HC595_DS);
	HC595_AssignPin(&valve, HC595_OE_GPIO_Port, HC595_OE_Pin, HC595_OE);
	HC595_ClearByteOutput(0xff);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_Reset()
{
	valve_BothRelease();
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
	// hạ cánh tay trái và phải
	HC595_SetBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_SetBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
	HC595_SetBitOutput(VALVE_HAND_LEFT_HC595_PIN); // kẹp tay gắp trái
	HC595_SetBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // kẹp tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
	// nâng cánh tay trái và phải
	HC595_ClearBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_ClearBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
}

void valve_BothRelease()
{
	// hạ cánh tay trái và phải
	HC595_SetBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_SetBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
	HC595_ClearBitOutput(VALVE_HAND_LEFT_HC595_PIN); // mở tay gắp trái
	HC595_ClearBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // mở tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
	// nâng cánh tay trái và phải
	HC595_ClearBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_ClearBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
}

void valve_ArmDownAndHandHold()
{
	// hạ cánh tay trái và phải
	HC595_SetBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_SetBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_ArmUp()
{
	// hạ cánh tay trái và phải
	HC595_ClearBitOutput(VALVE_ARM_LEFT_HC595_PIN);
	HC595_ClearBitOutput(VALVE_ARM_RIGHT_HC595_PIN);
	HC595_ShiftOut(NULL, 1, 1);
}

void valve_HandRelease()
{
	HC595_ClearBitOutput(VALVE_HAND_LEFT_HC595_PIN); // mở tay gắp trái
	HC595_ClearBitOutput(VALVE_HAND_RIGHT_HC595_PIN); // mở tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
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
