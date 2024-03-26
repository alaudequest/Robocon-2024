/*
 * HX711.h
 *
 *  Created on: Mar 31, 2023
 *      Author: Dien
 */
#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "main.h"
#ifdef CONFIG_USE_HX711

typedef GPIO_TypeDef HX711_Port_t;
typedef uint16_t HX711_Pin_t ;
typedef uint32_t HX711_EXTI_LINE_t ;

typedef struct{
	HX711_Port_t *port;
	HX711_Pin_t pin;
}pinconfig;

typedef struct{
	pinconfig SCK;
	pinconfig Data;
	uint32_t buffer;
	HX711_EXTI_LINE_t EXTI_LINE;
}HX711;

typedef enum{
	HX711_SCK,
	HX711_Data,
}pinName;

typedef enum{
	HX711_OK,
	HX711_ERROR,
	HX711_UNVALID_ARG,
}HX711_Status_t;



#define clkPort devTemp->SCK.port
#define clkPin devTemp->SCK.pin
#define dtPort devTemp->Data.port
#define dtPin devTemp->Data.pin
#define buffer devTemp->buffer
#define HX711_EXTI_line devTemp->EXTI_LINE

#define HX711_ENABLE_DOUT_INTERRUPT (HAL_NVIC_EnableIRQ(HX711_EXTI_line))
#define HX711_DISABLE_DOUT_INTERRUPT (HAL_NVIC_DisableIRQ(HX711_EXTI_line))
#define HX711_SCK_WRITE(LOGIC) (HAL_GPIO_WritePin(clkPort,clkPin,(LOGIC)))
#define HX711_DATA_READ (HAL_GPIO_ReadPin(dtPort,dtPin))

#define HX711_CALIB_OFFSET
#define HX711_AVERAGE_SAMPLE 10

HX711_Status_t HX711_AssignPin(HX711* dev,GPIO_TypeDef *port,uint16_t pin, pinName pinName);
HX711_Status_t HX711_AssignEXTILine(HX711* dev,IRQn_Type IRQ);
uint32_t HX711_ReadValue();
uint32_t HX711_ReadAverage();
void HX711_SetTarget(HX711* dev);
#endif /* INC_HX711_H_ */
#endif
