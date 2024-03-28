/*
 * HX711.c
 *
 *  Created on: Mar 31, 2023
 *      Author: Dien
 */


/**
 * Một số lỗi khi sử dụng thư viện:
 * Chưa khởi tạo biến đối tượng HX711, trả về HAL_ERROR
 * Tắt sai LINE ngắt
 * Chưa bật LINE ngắt
 *
 */

#include "HX711.h"

#ifdef CONFIG_USE_HX711

HX711* devTemp;

HX711_Status_t HX711_AssignPin(HX711* dev,GPIO_TypeDef *port,uint16_t pin, pinName pinName)
{
	if(!dev) return HAL_ERROR;
	switch(pinName){
	case HX711_SCK:
		dev->SCK.port = port;
		dev->SCK.pin = pin;
		break;
	case HX711_Data:
		dev->Data.port = port;
		dev->Data.pin = pin;
		break;
	}
	devTemp = dev;
	return HAL_OK;
}

HX711_Status_t HX711_AssignEXTILine(HX711* dev,IRQn_Type IRQ)
{
	if(!dev) return HX711_ERROR;
	dev->EXTI_LINE = IRQ;
	return HX711_OK;
}

void HX711_SetTarget(HX711* dev)
{
	while(!dev->EXTI_LINE);
	devTemp = dev;
}

uint32_t HX711_ReadValue()
{
	while(!devTemp->EXTI_LINE);
	HX711_DISABLE_DOUT_INTERRUPT;
	HX711_SCK_WRITE(0);
	buffer = 0;
	for (int i = 0; i < 24; i++){
		HX711_SCK_WRITE(1);
		buffer <<= 1;
		HX711_SCK_WRITE(0);
		if (HX711_DATA_READ) buffer++;
	}
	HX711_SCK_WRITE(1);
	buffer ^= 0x8000;
	HX711_SCK_WRITE(0);
	HX711_ENABLE_DOUT_INTERRUPT;
	return buffer;
}
uint32_t HX711_ReadAverage()
{
	static uint32_t Val=0,ValTemp;
	static uint8_t count=0;
	if(count < HX711_AVERAGE_SAMPLE){
		ValTemp += HX711_ReadValue();
		count++;
	} else {
		ValTemp/=HX711_AVERAGE_SAMPLE;
		Val = ValTemp;
		count = 0;
	}
	return Val;
}
#endif
