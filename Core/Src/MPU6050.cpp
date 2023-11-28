/*
 * MPU6050.cpp
 *
 *  Created on: Nov 28, 2023
 *      Author: KHOA
 */

#include "MPU6050.h"

void MPU6050::Init(I2C_HandleTypeDef *i2c, bool addressHigh) {
	this->i2c = i2c;
	if (addressHigh)
		this->address = MPU6050_ADDRESS_AD0_HIGH;
	else
		this->address = MPU6050_ADDRESS_AD0_LOW;
	this->initValid = 1;
}

HAL_StatusTypeDef MPU6050::IsReady() {
	if (!this->initValid) return HAL_ERROR;
	return HAL_I2C_IsDeviceReady(this->i2c, this->address << 1, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050::Write(MPU6050_RegisterAddress reg, uint8_t data) {
	if (!this->initValid) return HAL_ERROR;
	uint8_t dataTemp[2] = { reg, data };
	return HAL_I2C_Master_Transmit(this->i2c, this->address << 1, dataTemp, 2, HAL_MAX_DELAY);

}

uint8_t MPU6050::Read(MPU6050_RegisterAddress reg) {
	while (!this->initValid);
	uint8_t data[1];
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, data, 1, HAL_MAX_DELAY);
	return data[0];
}

uint16_t MPU6050::Read16(MPU6050_RegisterAddress reg) {
	while (!this->initValid);
	uint8_t data[2];
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, data, 2, HAL_MAX_DELAY);
	return data[0] << 8 | data[1];
}

int16_t MPU6050::GetTemperature() {
	int16_t data = (this->Read(TEMP_OUT_H) << 8 | this->Read(TEMP_OUT_L));
	return data;
}

