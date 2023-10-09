
/*
 * TB6600.h
 *
 *  Created on: Mar 28, 2023
 *      Author: SpiritBoi
 */
#ifndef _TB6600_H_
#define _TB6600_H_

#include "main.h"
#ifdef CONFIG_USE_TB6600
typedef struct pin{
	GPIO_TypeDef *port;
	uint16_t pin;
}pin;

typedef struct TB6600{
	pin pulse;
	pin dir;
	pin en;
}TB6600;

typedef enum {
	ACTIVE,
	INACTIVE,
	DIR_CCW,
	DIR_CW,
}StepState;

typedef enum {
	TB6600_PULSE,
	TB6600_DIR,
	TB6600_EN,
}pinName;

#define pulsePort devTemp->pulse.port
#define pulsePin devTemp->pulse.pin
#define dirPort devTemp->dir.port
#define dirPin devTemp->dir.pin
#define enPort devTemp->en.port
#define enPin devTemp->en.pin

#define TB6600_WRITE(PIN,LOGIC) ( 	((PIN) == TB6600_PULSE) ? 	HAL_GPIO_WritePin(pulsePort,pulsePin,(LOGIC)) 	:	\
									((PIN) == TB6600_DIR) 	?	HAL_GPIO_WritePin(dirPort,dirPin,(LOGIC)) 		: 	\
									((PIN) == TB6600_EN) 	?	HAL_GPIO_WritePin(enPort,enPin,(LOGIC)) 		: 0	\
								)

HAL_StatusTypeDef TB6600_AssignPin(TB6600* dev,GPIO_TypeDef *port,uint16_t pin, pinName pinName);
HAL_StatusTypeDef TB6600_SetDir(StepState dir);
HAL_StatusTypeDef TB6600_SetActive(StepState en);
HAL_StatusTypeDef TB6600_StepAuto(GPIO_PinState state);
HAL_StatusTypeDef TB6600_SetTarget(TB6600* dev);
#endif
#endif
