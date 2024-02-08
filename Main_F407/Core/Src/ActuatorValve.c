/*
 * ActuatorValve.c
 *
 *  Created on: Jan 15, 2024
 *      Author: SpiritBoi
 */

#include "ActuatorValve.h"
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
	HC595_SetBitOutput(1); // hạ cánh tay trái
	HC595_SetBitOutput(3); // hạ cánh tay phải
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
	HC595_SetBitOutput(0); // kẹp tay gắp trái
	HC595_SetBitOutput(2); // kẹp tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
	HC595_ClearBitOutput(1); // nâng cánh tay trái
	HC595_ClearBitOutput(3); // nâng cánh tay phải
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(500);
}
void valve_BothHold(){
	HC595_SetBitOutput(1); // hạ cánh tay trái
	HC595_SetBitOutput(3); // hạ cánh tay phải
	HC595_ShiftOut(NULL, 1, 1);
}
void valve_BothRelease()
{
//	HC595_SetBitOutput(1); // hạ cánh tay trái
//	HC595_SetBitOutput(3); // hạ cánh tay phải
//	HC595_ShiftOut(NULL, 1, 1);
//	osDelay(500);
	HC595_ClearBitOutput(0); // mở tay gắp trái
	HC595_ClearBitOutput(2); // mở tay gắp phải
	HC595_ShiftOut(NULL, 1, 1);
	osDelay(300);
	HC595_ClearBitOutput(1); // nâng cánh tay trái
	HC595_ClearBitOutput(3); // nâng cánh tay phải
	HC595_ShiftOut(NULL, 1, 1);
//	osDelay(500);
}

void valve_TestPin(pinName pin) {
	HC595_TestPin(pin);
}
