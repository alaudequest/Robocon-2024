/*
 * LogPort.h
 *
 *  Created on: Dec 8, 2023
 *      Author: KHOA
 */

#ifndef INC_LOGPORT_H_
#define INC_LOGPORT_H_

#include "main.h"
#include <string>
#include <cstring>

class UART_Log {
	private:
		std::string s;
		UART_HandleTypeDef *huart;

	public:
		template<class val> HAL_StatusTypeDef addParam(char *paramName, val inputData) {
			if (s.length() > 200) return HAL_ERROR;
			s += paramName;
			s += ':';
			s += std::to_string(inputData);
			s += '\t';
			return HAL_OK;
		}
		void begin(UART_HandleTypeDef *huart);
		void print();
};

#endif /* INC_LOGPORT_H_ */
