/*
 * LogPort.cpp
 *
 *  Created on: Dec 8, 2023
 *      Author: KHOA
 */

#include "LogPort.h"

void UART_Log::begin(UART_HandleTypeDef *huart) {
	this->huart = huart;
}

void UART_Log::print() {
	if (huart == NULL) return;
	s += '\n';
	HAL_UART_Transmit(huart, (uint8_t*) s.c_str(), s.length(), HAL_MAX_DELAY);
	s = "";
}

