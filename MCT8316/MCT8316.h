/*
 * MCT8316.h
 *
 *  Created on: Aug 16, 2023
 *      Author: SpiritBoi
 */

#ifndef MCT8316_H
#define MCT8316_H
#include "main.h"

#define CTRL_WORD_CRC_POS 22
#define CRC_ENABLE (1UL << CTRL_WORD_CRC_POS)
#define CRC_DISABLE (0UL << CTRL_WORD_CRC_POS)

#define DATA_LENGTH_16BIT 2
#define DATA_LENGTH_32BIT 4
#define DATA_LENGTH_64BIT 8

#define ISD_CONFIG 0x80
#define MOTOR_STARTUP1 0x82
#define MOTOR_STARTUP2 0x84
#define CLOSED_LOOP1 0x86
#define CLOSED_LOOP2 0x88
#define CLOSED_LOOP3 0x8a
#define CLOSED_LOOP4 0x8c
#define CONST_SPEED 0x8e
#define CONST_PWR 0x90
#define FAULT_CONFIG1 0x92
#define FAULT_CONFIG2 0x94
#define DEG_150_TWO_PH_PROFILE 0x96
#define DEG_150_THREE_PH_PROFILE 0x98
#define TRAP_CONFIG1 0x9a
#define TRAP_CONFIG2 0x9c
#define PIN_CONFIG1 0xa4
#define PIN_CONFIG2 0xa6
#define DEVICE_CONFIG 0xa8
#define GD_CONFIG1 0xac
#define GD_CONFIG2 0xae

#define ALGO_CTRL1 0xe6
#define DEVICE_CTRL 0xe8

typedef struct ControlWord {
	uint8_t rw;
	uint8_t crcEn;
	uint8_t dataLen;
}ControlWordConfig;


typedef struct MCT8316_t{
	uint8_t Data[4];
	uint8_t ctrlWord[3];
	I2C_HandleTypeDef *i2c;
	ControlWordConfig ctrlWordCfg;
}MCT8316_t;


HAL_StatusTypeDef MCT8316_IsReady(MCT8316_t *mct, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MCT8316_PackageControlWord(MCT8316_t *mct);
HAL_StatusTypeDef MCT8316_Write_64bit(MCT8316_t *mct, uint8_t reg, uint64_t data);
HAL_StatusTypeDef MCT8316_Write(MCT8316_t *mct, uint8_t reg, uint32_t data);
uint32_t MCT8316_Read(MCT8316_t *mct, uint8_t reg);
HAL_StatusTypeDef MCT8316_EEPROM_Read(MCT8316_t *mct);
HAL_StatusTypeDef MCT8316_EEPROM_Write(MCT8316_t *mct);
HAL_StatusTypeDef MCT8316_EEPROM_Commit(MCT8316_t *mct);
HAL_StatusTypeDef MCT8316_SetSpeed(MCT8316_t *mct,uint8_t speedPercent);
HAL_StatusTypeDef MCT8316_SetOverride(MCT8316_t *mct, uint8_t override);


#endif
