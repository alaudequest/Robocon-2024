/*
 * MCT8316.c
 *
 *  Created on: Aug 16, 2023
 *      Author: KHOA
 */

#include "MCT8316.h"
#include "MCT8316_Define.h"
#include "ALGO_CTRL1_Register.h"
#include "ISD_CONFIG_Register.h"
#include "DEVICE_CTRL_Register.h"
HAL_StatusTypeDef MCT8316_IsReady(MCT8316_t *mct, I2C_HandleTypeDef *hi2c)
{
	mct->i2c = hi2c;
	if(!mct || !mct->i2c) return HAL_ERROR;
	return HAL_I2C_IsDeviceReady(mct->i2c, 0x00, 1, HAL_MAX_DELAY);
	return HAL_ERROR;
}


HAL_StatusTypeDef MCT8316_PackageControlWord(MCT8316_t *mct)
{
	if(!mct) return HAL_ERROR;
	uint32_t Temp = 0;
	// the default setup is WRITE_MODE, CRC_DISABLE, DATA_LENGTH_16BIT
	if(mct->ctrlWordCfg.rw) Temp |= MCT_READ_MODE;
	if(mct->ctrlWordCfg.crcEn) Temp |= CRC_ENABLE;
	else if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_32BIT) Temp |= DLEN_32BIT;
	else if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_64BIT) Temp |= DLEN_64BIT;
	mct->ctrlWord[1] =  (Temp & 0xff00) >> 8;
	mct->ctrlWord[0] =  (Temp & 0xff0000) >> 16;
	return HAL_OK;

}

HAL_StatusTypeDef MCT8316_Write(MCT8316_t *mct, uint8_t reg, uint32_t data)
{
	if(!mct || !mct->i2c) return HAL_ERROR;
	HAL_StatusTypeDef err = HAL_OK;
	mct->ctrlWordCfg.rw = WRITE_MODE;
	mct->ctrlWord[2] = reg;
	if(data & 0xffff0000) mct->ctrlWordCfg.dataLen = DATA_LENGTH_32BIT;
	else mct->ctrlWordCfg.dataLen = DATA_LENGTH_16BIT;

	mct->Data[0] =  data & 0xff;
	mct->Data[1] = (data & 0xff00) >> 8;
	mct->Data[2] = (data & 0xff0000) >> 16;
	mct->Data[3] = (data & 0xff000000) >> 24;

	err |= MCT8316_PackageControlWord(mct);

	err |= HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->ctrlWord, 3, HAL_MAX_DELAY);
	if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_32BIT) err |= HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->Data, 4, HAL_MAX_DELAY);
	else err |= HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->Data, 2, HAL_MAX_DELAY);

	if(err != HAL_OK) return err;
	return HAL_OK;
}

HAL_StatusTypeDef MCT8316_Write_64bit(MCT8316_t *mct, uint8_t reg, uint64_t data)
{
	if(!mct || !mct->i2c) return HAL_ERROR;
	HAL_StatusTypeDef err = HAL_OK;
	mct->ctrlWordCfg.rw = WRITE_MODE;
	mct->ctrlWord[2] = reg;
	mct->ctrlWordCfg.dataLen = DATA_LENGTH_64BIT;

	err |= MCT8316_PackageControlWord(mct);

	err |= HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->ctrlWord, 3, HAL_MAX_DELAY);
	err |= HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->Data, 8, HAL_MAX_DELAY);

	if(err != HAL_OK) return err;
	return HAL_OK;
}

uint32_t MCT8316_Read(MCT8316_t *mct, uint8_t reg)
{
	uint32_t data = 0;
	mct->ctrlWordCfg.rw = READ_MODE ;
	mct->ctrlWord[2] = reg;
	mct->ctrlWordCfg.dataLen = DATA_LENGTH_32BIT;
	MCT8316_PackageControlWord(mct);

	HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->ctrlWord, 3, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(mct->i2c, 0x00, mct->Data, 4,HAL_MAX_DELAY);

	data |= mct->Data[0];
	data |= mct->Data[1] << 8;
	data |= mct->Data[2] << 16;
	data |= mct->Data[3] << 24;
	return data;
}

HAL_StatusTypeDef MCT8316_EEPROM_Read(MCT8316_t *mct)
{
	HAL_StatusTypeDef err = HAL_OK;
	err |= MCT8316_Write(mct, ALGO_CTRL1, EEPROM_READ);
	HAL_Delay(100);
	return err;
}

HAL_StatusTypeDef MCT8316_EEPROM_Write(MCT8316_t *mct)
{
	HAL_StatusTypeDef err = HAL_OK;
	err |= MCT8316_Write(mct, ALGO_CTRL1, EEPROM_WRITE);
	HAL_Delay(100);
	return err;
}

HAL_StatusTypeDef MCT8316_EEPROM_Commit(MCT8316_t *mct)
{
	HAL_StatusTypeDef err = HAL_OK;
	err = MCT8316_Write(mct, ALGO_CTRL1,EEPROM_WRITE_ACCESS_KEY_CODE);
	HAL_Delay(100);
	return err;
}
