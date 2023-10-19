/*
 * 74HC165.c
 *
 * Created on: Apr 30, 2023
 * 		Author: MinhTri059
 */

#include "74HC165.h"
#ifdef CONFIG_USE_74HC165

HC165 *_hc165 = NULL;
HC165_Status_t HC165_AssignPin(HC165 *hc165, GPIO_TypeDef *port, uint16_t pin, HC165_PinName pinName){
	if (!hc165) return HC165_INVALID_ARG;
	switch (pinName){
	case HC165_CP:
		hc165->CP.Port = port;
		hc165->CP.Pin = pin;
		break;
	case HC165_CE:
		hc165->CE.Port = port;
		hc165->CE.Pin = pin;
		break;
	case HC165_PL:
		hc165->PL.Port = port;
		hc165->PL.Pin = pin;
		break;
	case HC165_DATA:
		hc165->DATA.Port = port;
		hc165->DATA.Pin = pin;
	}
	_hc165 = hc165;
	return HC165_OK;
}

uint32_t HC165_ReadState(uint8_t n)
{
	uint32_t data;
	if (n > HC165_MAX_CASCADE)	return HC165_BEYOND_MAX_CASCADE;
	if (!_hc165 && !n)	return HC165_INVALID_ARG;
	data = 0;
	uint8_t value = 0;
	HC165_WRITE(HC165_PL,0);
	HC165_WRITE(HC165_PL,1);
	for (int i = 0; i < 8*n; i++){
		value = HC165_READ;
		data |= value << ((8*n - 1) - i);
		HC165_WRITE(HC165_CP,1);
		HC165_WRITE(HC165_CP,0);
	}
	return data;
}

#endif
