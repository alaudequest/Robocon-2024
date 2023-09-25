/*
 * 74hc595.c
 *
 *  Created on: Sep 23, 2023
 *      Author: NamDHay
 */


#include "74hc595.h"
HC595 *hc595 = NULL;
uint32_t hc595event;
void hc595_TestAllOutput()
{
	HAL_GPIO_WritePin(HC595_DSI_GPIO_Port, HC595_DSI_Pin, 1);
	for(uint8_t i = 0;i<8;i++){
		HAL_GPIO_WritePin(HC595_CLK_GPIO_Port, HC595_CLK_Pin, 1);
		HAL_GPIO_WritePin(HC595_CLK_GPIO_Port, HC595_CLK_Pin, 0);
	}
	HAL_GPIO_WritePin(HC595_RCK_GPIO_Port, HC595_RCK_Pin, 1);
	HAL_GPIO_WritePin(HC595_RCK_GPIO_Port, HC595_RCK_Pin, 0);
}
void HC595_AsignPin(pinName pin){
	switch(pin){
	case HC595_DSI:
		break;
	case HC595_CLK:
		break;
	case HC595_RCK:
		break;
	case HC595_OE:
		break;
	}
}
void hc595_ShiftOut(void* data)
{
	for(uint8_t i = 0;i<8;i++){
		if(!(data&0x01))	HAL_GPIO_WritePin(HC595_DSI_GPIO_Port, HC595_DSI_Pin, 0);
		else	HAL_GPIO_WritePin(HC595_DSI_GPIO_Port, HC595_DSI_Pin, 1);
		HAL_GPIO_WritePin(HC595_CLK_GPIO_Port, HC595_CLK_Pin, 1);
		HAL_GPIO_WritePin(HC595_CLK_GPIO_Port, HC595_CLK_Pin, 0);
		data = data >> 1;
	}
	HAL_GPIO_WritePin(HC595_RCK_GPIO_Port, HC595_RCK_Pin, 1);
	HAL_GPIO_WritePin(HC595_RCK_GPIO_Port, HC595_RCK_Pin, 0);
}
void hc595_SetOutput(vanNum vanNum)
{
	hc595->data|=(1UL<<vanNum);
}
void hc595_ClearOutput(vanNum vanNum)
{
	hc595->data&=~(1UL<<vanNum);
}
void hc595_OutputEnable()
{
	HAL_GPIO_WritePin(HC595_OE_GPIO_Port, HC595_OE_Pin, 0);
}
void hc595_OutputDisable()
{
	HAL_GPIO_WritePin(HC595_OE_GPIO_Port, HC595_OE_Pin, 1);
}
