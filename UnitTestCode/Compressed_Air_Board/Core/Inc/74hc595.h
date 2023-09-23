/*
 * 74hc595.h
 *
 *  Created on: Sep 23, 2023
 *      Author: NamDHay
 */

#ifndef INC_74HC595_H_
#define INC_74HC595_H_
#include "main.h"

#define HC595_SETFLAG(flag)		(hc595event |= (1<<flag))
#define HC595_CLEARFLAG(flag)	(hc595event &= ~(1<<flag))
#define HC595_CHECKFLAG(flag)	((hc595event & (1<<flag)) == (1<<flag) ? 1 : 0)
typedef enum{
	VAN1,
	VAN2,
	VAN3,
	VAN4,
	VAN5,
	VAN6,
	VAN7,
	VAN8,
}VanNum;
typedef struct HC595{
	uint8_t data;
}HC595;

void hc595_TestAllOutput();
void hc595_ShiftOut(uint8_t data);
void hc595_SetOutput(uint8_t pos);
void hc595_ClearOutput(uint8_t pos);
void hc595_OutputEnable();
void hc595_OutputDisable();
#endif /* INC_74HC595_H_ */

