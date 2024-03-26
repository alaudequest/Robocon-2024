/*
 * 74HC595.c
 *
 *  Created on: Mar 23, 2023
 *      Author: hoanganh
 */

#include "74HC595.h"
#ifdef CONFIG_USE_74HC595
HC595 *_hc595 = NULL;
HC595_Status_t HC595_AssignPin(HC595* hc595,GPIO_TypeDef *port,uint16_t pin, pinName pinName)
{
	if(!hc595) return HC595_INVALID_ARG;
	switch(pinName){
	case HC595_CLK:
		hc595->clk.port = port;
		hc595->clk.pin = pin;
		break;
	case HC595_DS:
		hc595->ds.port = port;
		hc595->ds.pin = pin;
		break;
	case HC595_LATCH:
		hc595->latch.port = port;
		hc595->latch.pin = pin;
		break;
	case HC595_OE:
		hc595->oe.port = port;
		hc595->oe.pin = pin;
		break;
	}
	_hc595 = hc595;
	return HC595_OK;
}

static inline uint32_t ByteMerging(const uint8_t *byte_ptr, bool LSB_FIRST) {
    uint32_t result = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        uint8_t index = LSB_FIRST ? i : (3 - i);
        result |= ((uint32_t)byte_ptr[index]) << (i * 8);
    }
    return result;
}


/* value = 0x12345678
 * LSB_FIRST = 1
 * byte_array[0] = 0x78
 * byte_array[1] = 0x56
 * byte_array[2] = 0x34
 * byte_array[3] = 0x12
 * LSB_FIRST = 0
 * byte_array[3] = 0x78
 * byte_array[2] = 0x56
 * byte_array[1] = 0x34
 * byte_array[0] = 0x12
 */
static inline void ByteSlitting(uint32_t value, uint8_t *byte_array, bool LSB_FIRST)
{
    for (uint8_t i = 0; i < 4; i++){
        uint8_t index = LSB_FIRST ? i : (3 - i);
        byte_array[index] = (uint8_t)((value >> (8 * i)) & 0xFF);
    }
}


/**
 * @brief Sends data to cascaded 74HC595 shift registers.
 * This function shifts data into multiple cascaded 74HC595 registers.
 *
 * Convention:
 * The first 74HC595[0] that connected to MCU via pin 14(DS or serial data pin), pin 9 connected
 * to the next 74HC595.
 * The last 74HC595[n] pin 9 is not connect, pin 14 connected to previous 74HC595.
 *
 * @param dt Pointer to an array of uint8_t data to be shifted into the registers.
 * @param n Number of cascaded 74HC595 shift registers.
 * @param Device0GetLSBByte send MSB byte (n-1) to the last HC595[n],
 * LSB byte (byte 0) to the first HC595[0]
 * The first byte data will send to the last 74HC595[n]
 * The last byte data will send to the first 74HC595[0].
 * @return HC595_Status_t status code indicating the result of the operation.
 */
HC595_Status_t HC595_ShiftOut(uint8_t *dt,uint8_t n,uint8_t Device0GetLSBByte)
{
	if(!_hc595 || !n) return HC595_INVALID_ARG;
	if(n > HC595_MAX_CASCADE) return HC595_BEYOND_MAX_CASCADE;
	// Use internal buffer to shift out if dt is NULL
	uint8_t a;
	if(!dt && n < 5) {
	    uint8_t Temp[4];
	    ByteSlitting(_hc595->data,Temp,Device0GetLSBByte);
	    dt = Temp;
	}
	a = *(dt+(n-1));
	for(int8_t i=(n*8)-1; i > -1; i--){
	    if(a&0x80) HC595_WRITE(HC595_DS,1);
	    else HC595_WRITE(HC595_DS,0);
		HC595_WRITE(HC595_CLK,0);
		HC595_WRITE(HC595_CLK,0);
		HC595_WRITE(HC595_CLK,1);
		HC595_WRITE(HC595_CLK,1);
		a <<= 1;
	    if(i%8 == 0) {
	        a = *(dt+(i/8-1));
	    }
	}
	HC595_WRITE(HC595_LATCH,1);
	HC595_WRITE(HC595_LATCH,0);
	return HC595_OK;
}



void HC595_ReturnValueToString(char *s,HC595_Status_t retVal){
    switch(retVal){
        case HC595_OK: strcpy(s,"HC595_OK"); break;
        case HC595_BEYOND_MAX_CASCADE: strcpy(s,"HC595_BEYOND_MAX_CASCADE"); break;
        case HC595_INVALID_ARG: strcpy(s,"HC595_INVALID_ARG"); break;
        case HC595_ERROR: strcpy(s,"HC595_ERROR"); break;
    }
}


void HC595_SetBitOutput(uint8_t pos)
{
	_hc595->data|=(1UL<<pos);
}

void HC595_ClearBitOutput(uint8_t pos)
{
	_hc595->data&=~(1UL<<pos);
}

void HC595_SetByteOutput(uint32_t value)
{
	_hc595->data|=value;
}

void HC595_ClearByteOutput(uint32_t value)
{
	_hc595->data&=~(value);
}

HC595_Status_t HC595_SetTarget(HC595 *hc595)
{
	if(!hc595) return HC595_ERROR;
	_hc595 = hc595;
	return HC595_OK;
}

void HC595_TestOutput()
{
	static uint16_t dataOriginal= 0x0200;
	static uint8_t shft=0;
	uint8_t data[2];
	shft++;
	dataOriginal>>=1;
	dataOriginal|=0x0200;
	if(shft==10) {
		shft = 0;
		dataOriginal = 0x0200;
	}
	data[0]=dataOriginal >> 8;
	data[1]=dataOriginal;
	HC595_ShiftOut(data,2,true);
	DELAY_MS(100);
}

void HC595_TestPin(pinName pinName)
{
	while(!_hc595);
	HC595_WRITE(pinName,1);
	DELAY_MS(2000);
	HC595_WRITE(pinName,0);
	DELAY_MS(2000);
}

void HC595_EnableOutput()
{
	HC595_WRITE(HC595_OE,0);
}

void HC595_DisableOutput()
{
	HC595_WRITE(HC595_OE,1);
}
#endif

