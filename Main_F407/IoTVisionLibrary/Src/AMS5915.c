/*
 * AMS5915.c
 *
 *  Created on: Mar 26, 2023
 *      Author: SpiritBoi
 */
#include "AMS5915.h"
#ifdef CONFIG_USE_AMS5915
AMS5915 *_ams;
/**
 * @brief Initializes the AMS5915 pressure sensor.
 *
 * This function initializes and configures the AMS5915 pressure sensor
 * with the given I2C_HandleTypeDef instance for communication.
 *
 * @param ams Pointer to an AMS5915 struct to handle pressure sensor operations.
 * @param hi2c Pointer to an I2C_HandleTypeDef instance representing the I2C line connected to the pressure sensor.
 * @return AMS5915_Status_t value indicating the result of the initialization operation.
 */
AMS5915_Status_t AMS5915_Init(AMS5915 *ams, I2C_HandleTypeDef *hi2c)
{
	if(!ams && !hi2c) return AMS5915_INVALID_ARG;
	ams->hi2c = hi2c;
	_ams = ams;
	while(AMS5915_CHECK_DEVICE_READY!=HAL_OK);
	return AMS5915_OK;
}


AMS5915_Status_t AMS5915_SetTarget(AMS5915 *ams)
{
	if(!ams) return AMS5915_INVALID_ARG;
	_ams = ams;
	return AMS5915_OK;
}

/**
 * @brief Read raw data from AMS5915 pressure sensor.
 * @param ams Pointer to an AMS5915 struct to handle pressure sensor operations.
 * @return AMS5915_Status_t
 * @note Need to use void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) in main
 */
AMS5915_Status_t AMS5915_ReadRaw(AMS5915 *ams)
{
	if(!ams && !_ams) return AMS5915_INVALID_ARG;
	if(ams) _ams = ams;
	if(AMS5915_READ_RAW_DATA == HAL_OK) return AMS5915_OK;
	return AMS5915_OK;
}

/**
 * @brief Calculate pressure value from raw data stored in AMS5915 struct buf member.
 * @param ams Pointer to an AMS5915 struct to handle pressure sensor operations.
 * @return double
 */
float AMS5915_CalPressure(AMS5915 *ams)
{
	uint16_t sensp ;
	float p, digout_p;
	while(AMS5915_ReadRaw(ams) != AMS5915_OK);
	sensp = (AMS5915_DIGOUT_PMAX - AMS5915_DIGOUT_PMIN) / (AMS5915_PMAX - AMS5915_PMIN);
	digout_p = ((ams->buf[0] & 0x3f) << 8) | (ams->buf[1]);
	p = (digout_p - AMS5915_DIGOUT_PMIN) / sensp + AMS5915_PMIN;
	return p*100;
}

#endif /* defined(CONFIG_USE_AMS5915) */
