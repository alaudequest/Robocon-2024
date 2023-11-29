/*
 * MPU6050.h
 *
 *  Created on: Nov 28, 2023
 *      Author: KHOA
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "main.h"
#include "MPU6050_RegisterDef.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define INIT_NOT_VALID 0
#define INIT_VALID 1

#ifdef __cplusplus

class MPU6050 {
	private:
		I2C_HandleTypeDef *i2c;
		uint8_t address;
		uint8_t initValid;

	public:
		void init(I2C_HandleTypeDef *i2c, bool addressHigh);
		HAL_StatusTypeDef write(MPU6050_RegisterAddress reg, uint8_t data);
		uint8_t read(MPU6050_RegisterAddress reg);
		int16_t readSignHalfWord(MPU6050_RegisterAddress reg);
		HAL_StatusTypeDef writeBurst(MPU6050_RegisterAddress reg, uint8_t *wdata, uint8_t writeTime);
		HAL_StatusTypeDef readBurst(MPU6050_RegisterAddress reg, uint8_t *rdata, uint8_t readTime);
		HAL_StatusTypeDef isReady();
		float getTemperature();
		void setSampleRateDivider(uint8_t div);
		uint8_t getSampleRateDivider();
		void setClockSource(MPU6050_ClockSource clksource);
		MPU6050_ClockSource getClockSource();
		void setLowPassFilterBandwidth(MPU6050_DigitalLowPassFilterBandwidth bw);
		MPU6050_DigitalLowPassFilterBandwidth getLowPassFilterBandwidth();
		int16_t getRotationZ();

};

#endif

#endif /* INC_MPU6050_H_ */
