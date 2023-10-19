/*
 * 74HC595.h
 *
 *  Created on: Mar 23, 2023
 *      Author: hoanganh
 */


#ifndef INC_74HC595_H_
#define INC_74HC595_H_
#include "main.h"
#ifdef CONFIG_USE_74HC595
#include "string.h"
#include "stdbool.h"

#define latchPin _hc595->latch.pin
#define clkPin _hc595->clk.pin
#define dsPin _hc595->ds.pin
#define oePin _hc595->oe.pin
#define latchPort _hc595->latch.port
#define clkPort _hc595->clk.port
#define dsPort _hc595->ds.port
#define oePort _hc595->oe.port

#define HC595_MAX_CASCADE 4

#define HC595_WRITE(PIN,LOGIC) ( 	((PIN) == HC595_LATCH)  ? 	HAL_GPIO_WritePin(latchPort,latchPin,(LOGIC)) 	:	\
									((PIN) == HC595_CLK) 	?	HAL_GPIO_WritePin(clkPort,clkPin,(LOGIC)) 		: 	\
									((PIN) == HC595_OE) 	?	HAL_GPIO_WritePin(oePort,oePin,(LOGIC)) 		: 	\
									((PIN) == HC595_DS) 	?	HAL_GPIO_WritePin(dsPort,dsPin,(LOGIC)) 		: 0	\
								)

#define DELAY_MS(X) (HAL_Delay(X))


typedef struct pinConfig{
	GPIO_TypeDef *port;
	uint16_t pin;
}pinConfig;

typedef struct HC595{
	pinConfig ds;
	pinConfig clk;
	pinConfig latch;
	pinConfig oe;
	uint32_t data;
}HC595;

typedef enum{
	HC595_LATCH,
	HC595_CLK,
	HC595_DS,
	HC595_OE
}pinName;

typedef enum{
	HC595_OK,
	HC595_ERROR,
	HC595_INVALID_ARG,
	HC595_BEYOND_MAX_CASCADE,
}HC595_Status_t;

HC595_Status_t HC595_ShiftOut(uint8_t *dt,uint8_t n,uint8_t MSB_FIRST);
HC595_Status_t HC595_AssignPin(HC595* dev,GPIO_TypeDef *port,uint16_t pin, pinName pinName);
HC595_Status_t HC595_SetTarget(HC595 *hc595);
void HC595_SetBitOutput(uint8_t pos);
void HC595_ClearBitOutput(uint8_t pos);
void HC595_SetByteOutput(uint32_t value);
void HC595_ClearByteOutput(uint32_t value);
void HC595_TestPin(pinName pin);
void HC595_DisableOutput();
void HC595_EnableOutput();
void HC595_TestOutput();
#endif /* INC_74HC595_H_ */
#endif

