/*
 *
 *	74HC165.h
 *
 *	Created on: Apr 28, 2023
 *		Author: MinhTri059
 */

#ifndef INC_74HC165_H_
#define INC_74HC165_H_

#include "main.h"
#ifdef CONFIG_USE_74HC165
#include "string.h"
#include "stdbool.h"

#define plPin		_hc165->PL.Pin	//Using datasheet of Nexperia to define pin name
#define cpPin		_hc165->CP.Pin
//#define dsPin		_hc165->DS.Pin
#define cePin		_hc165->CE.Pin
#define dataPin 	_hc165->DATA.Pin

#define plPort		_hc165->PL.Port
#define cpPort		_hc165->CP.Port
//#define dsPort		_hc165->DS.Port
#define cePort		_hc165->CE.Port
#define dataPort 	_hc165->DATA.Port

#define HC165_MAX_CASCADE	4

#define HC165_WRITE(PIN, LOGIC)	(	((PIN) == HC165_PL) 	? 	HAL_GPIO_WritePin(plPort, plPin, (LOGIC))	:	\
									((PIN) == HC165_CP)		? 	HAL_GPIO_WritePin(cpPort, cpPin, (LOGIC))	:	\
									((PIN) == HC165_CE)		?	HAL_GPIO_WritePin(cePort, cePin, (LOGIC))	: 	0	\
								)
#define HC165_READ (HAL_GPIO_ReadPin(dataPort, dataPin))
typedef struct HC165_PinConfig{
	GPIO_TypeDef *Port;
	uint16_t	Pin;
}HC165_PinConfig;

typedef struct HC165{
	HC165_PinConfig PL;
	HC165_PinConfig CP;
//	HC165_PinConfig DS;
	HC165_PinConfig CE;
	HC165_PinConfig DATA;
}HC165;

typedef enum{
	HC165_PL,
	HC165_CP,
//	HC165_DS,
	HC165_CE,
	HC165_DATA
}HC165_PinName;


typedef enum{
	HC165_OK,
	HC165_ERROR,
	HC165_INVALID_ARG,
	HC165_BEYOND_MAX_CASCADE
}HC165_Status_t;

HC165_Status_t HC165_AssignPin(HC165 *p, GPIO_TypeDef *port, uint16_t pin, HC165_PinName pinName);
uint32_t HC165_ReadState(uint8_t n);
#endif

#endif
