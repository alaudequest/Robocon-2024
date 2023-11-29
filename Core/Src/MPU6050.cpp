/*
 * MPU6050.cpp
 *
 *  Created on: Nov 28, 2023
 *      Author: KHOA
 */

#include "MPU6050.h"
#include "string.h"

void MPU6050::setClockSource(MPU6050_ClockSource clksource) {
	MPU6050_PowerManagement1 pwr1;
	uint8_t data;
	data = this->read(PWR_MGMT_1);
	memcpy(&pwr1, &data, 1);
	pwr1.clkSelect = clksource;
	memcpy(&data, &pwr1, 1);
	this->write(PWR_MGMT_1, data);
}

MPU6050_ClockSource MPU6050::getClockSource() {
	this->read(PWR_MGMT_1);
}

void MPU6050::setSampleRateDivider(uint8_t div) {
	this->write(SMPLRT_DIV, div);
}

uint8_t MPU6050::getSampleRateDivider() {
	return this->read(SMPLRT_DIV);
}

void MPU6050::setLowPassFilterBandwidth(MPU6050_DigitalLowPassFilterBandwidth bw) {
	MPU6050_Configuration cfg;
	uint8_t data;
	/* To avoid override data from other bit fields in the same register, only need to change specific bit field
	 * so first read data from sensor, then change only these field and write data back to sensor
	 */
	data = this->read(CONFIG);
	memcpy(&cfg, &data, 1);
	cfg.digitalLowPassFilter = bw;
	memcpy(&data, &cfg, 1);
	this->write(CONFIG, data);
}

MPU6050_DigitalLowPassFilterBandwidth MPU6050::getLowPassFilterBandwidth() {
	return (MPU6050_DigitalLowPassFilterBandwidth) (this->read(CONFIG) & 0x07); // 0x07 is mask bit of bit field LowPassFilterBandwidth
}

int16_t MPU6050::getRotationZ() {
	return this->readSignHalfWord(GYRO_ZOUT_H);
}

float MPU6050::getTemperature() {
	return ((float) this->readSignHalfWord(TEMP_OUT_H) / 340.0 + 36.53);
}

HAL_StatusTypeDef MPU6050::writeBurst(MPU6050_RegisterAddress reg, uint8_t *wdata, uint8_t writeTime) {
	if (!this->initValid) return HAL_ERROR;
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, wdata, writeTime, HAL_MAX_DELAY);
	return HAL_OK;

}

HAL_StatusTypeDef MPU6050::readBurst(MPU6050_RegisterAddress reg, uint8_t *rdata, uint8_t readTime) {
	if (!this->initValid) return HAL_ERROR;
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, rdata, readTime, HAL_MAX_DELAY);
	return HAL_OK;
}

void MPU6050::init(I2C_HandleTypeDef *i2c, bool addressHigh) {
	this->i2c = i2c;
	if (addressHigh)
		this->address = MPU6050_ADDRESS_AD0_HIGH;
	else
		this->address = MPU6050_ADDRESS_AD0_LOW;
	this->initValid = 1;
}

HAL_StatusTypeDef MPU6050::isReady() {
	if (!this->initValid) return HAL_ERROR;
	return HAL_I2C_IsDeviceReady(this->i2c, this->address << 1, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050::write(MPU6050_RegisterAddress reg, uint8_t data) {
	if (!this->initValid) return HAL_ERROR;
	uint8_t dataTemp[2] = { reg, data };
	return HAL_I2C_Master_Transmit(this->i2c, this->address << 1, dataTemp, 2, HAL_MAX_DELAY);

}

uint8_t MPU6050::read(MPU6050_RegisterAddress reg) {
	while (!this->initValid);
	uint8_t data[1];
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, data, 1, HAL_MAX_DELAY);
	return data[0];
}

int16_t MPU6050::readSignHalfWord(MPU6050_RegisterAddress reg) {
	while (!this->initValid);
	uint8_t data[2];
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, data, 2, HAL_MAX_DELAY);
	return data[0] << 8 | data[1];
}
