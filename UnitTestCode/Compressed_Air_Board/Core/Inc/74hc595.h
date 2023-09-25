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
//#define VAN1	(1<<7)
//#define VAN2	(1<<6)
//#define VAN3	(1<<5)
//#define VAN4	(1<<4)
//#define VAN5	(1<<3)
//#define VAN6	(1<<2)
//#define VAN7	(1<<1)
//#define VAN8	(1<<0)

typedef enum{
	VAN8,
	VAN7,
	VAN6,
	VAN5,
	VAN4,
	VAN3,
	VAN2,
	VAN1,
}vanNum;
typedef enum{
	HC595_DSI,
	HC595_CLK,
	HC595_RCK,
	HC595_OE,
}pinName;
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

