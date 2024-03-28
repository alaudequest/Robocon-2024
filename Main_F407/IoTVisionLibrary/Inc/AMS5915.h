/*
 * AMS5915.h
 *
 *  Created on: Mar 26, 2023
 *      Author: SpiritBoi
 */

#include "main.h"
#ifdef CONFIG_USE_AMS5915
#ifndef _AMS5915_H_
#define _AMS5915_H_

#define AMS5915_ADDR (0x28 << 1)
#define AMS5915_MIN_PRES_MBAR -50.0f
#define AMS5915_MAX_PRES_MBAR 50.0f
#define AMS5915_PMAX   50
#define AMS5915_PMIN  0
#define AMS5915_DIGOUT_PMIN 1638
#define AMS5915_DIGOUT_PMAX 14745
#define AMS5915_SIZE_BUF 4



#define AMS5915_CHECK_DEVICE_READY (HAL_I2C_IsDeviceReady(hi2c, AMS5915_ADDR, 3, 500))
#define AMS5915_READ_RAW_DATA (HAL_I2C_Master_Receive(ams->hi2c, AMS5915_ADDR, ams->buf, 4,HAL_MAX_DELAY))

typedef struct AMS5915{
	uint8_t buf[AMS5915_SIZE_BUF];
	I2C_HandleTypeDef *hi2c;
}AMS5915;

typedef enum{
	AMS5915_OK,
	AMS5915_ERROR,
	AMS5915_INVALID_ARG,
}AMS5915_Status_t;

AMS5915_Status_t AMS5915_Init(AMS5915 *ams, I2C_HandleTypeDef *hi2c);
AMS5915_Status_t AMS5915_SetTarget(AMS5915 *ams);
AMS5915_Status_t AMS5915_ReadRaw(AMS5915 *ams);
float AMS5915_CalPressure(AMS5915 *ams);

#endif
#endif /*  defined(CONFIG_USE_AMS5915) */
