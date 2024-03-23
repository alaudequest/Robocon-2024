/*
 * ParameterSetting.h
 *
 *  Created on: Mar 18, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_PARAMETERSETTING_H_
#define INC_PARAMETERSETTING_H_

#include "main.h"
#include "string.h"

typedef void (*pConsolePrint)(char*);

typedef enum NavigationCommandLineInterface {
	NAVCLI_AXIS,
	NAVCLI_CONTROL_TYPE,
	NAVCLI_STAGE,
} NavigationCommandLineInterface;

void navcli_Init(UART_HandleTypeDef *huart, void (*pConsolePrint)(char*));

#endif /* INC_PARAMETERSETTING_H_ */
