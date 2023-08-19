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
HAL_StatusTypeDef MCT8316_IsReady(MCT8316_t *mct)
{
	if(!mct || !mct->i2c) return HAL_ERROR;
	return HAL_I2C_IsDeviceReady(mct->i2c, 0x00, 1, 100);
	return HAL_ERROR;
}


HAL_StatusTypeDef MCT8316_PackageControlWord(MCT8316_t *mct, uint8_t reg)
{
	if(!mct) return HAL_ERROR;
	uint32_t Temp = 0;
	// the default setup is WRITE_MODE, CRC_DISABLE, DATA_LENGTH_16BIT
	if(mct->ctrlWordCfg.rw) Temp |= READ_MODE;
	if(mct->ctrlWordCfg.crcEn) Temp |= CRC_ENABLE;
	else if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_32BIT) Temp |= DLEN_32BIT;
	else if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_64BIT) Temp |= DLEN_64BIT;
	Temp |= reg;
	mct->ctrlWord[0] =  Temp & 0xff;
	mct->ctrlWord[1] =  (Temp & 0xff00) >> 8;
	mct->ctrlWord[2] =  (Temp & 0xff0000) >> 16;
	return HAL_OK;

}

//HAL_StatusTypeDef MCT8316_Write(MCT8316_t *mct, uint8_t *data, uint8_t len)
//{
//
//}

HAL_StatusTypeDef MCT8316_Read(MCT8316_t *mct, uint8_t reg)
{
	if(!mct || !mct->i2c) return HAL_ERROR;
	mct->ctrlWord[0] = reg;
	HAL_I2C_Master_Transmit(mct->i2c, 0x00, mct->ctrlWord, 3, 100);

	if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_64BIT)
	HAL_I2C_Master_Receive_IT(mct->i2c, 0x00, mct->Data, 8);
	else if(mct->ctrlWordCfg.dataLen == DATA_LENGTH_32BIT)
	HAL_I2C_Master_Receive_IT(mct->i2c, 0x00, mct->Data, 4);
	else HAL_I2C_Master_Receive_IT(mct->i2c, 0x00, mct->Data, 2);
	return HAL_OK;
}
