/*
 * CRC16.h
 *
 *  Created on: Jan 30, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_

#include "main.h"

uint16_t crc16_Unreflected(const uint8_t *buf, uint32_t len, uint16_t crc_init);


#endif /* INC_CRC16_H_ */
