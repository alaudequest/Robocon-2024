/*
 * MPU6050.cpp
 *
 *  Created on: Nov 28, 2023
 *      Author: KHOA
 */

#include "MPU6050.h"
#include "string.h"
// Methods for configuring config registers ****************************************************************************************************
void MPU6050::setFullScaleGyroRange(MPU6050_FullscaleRange fs) {
	uint8_t data;
	this->cfgReg.gycfg.fullscaleRange = fs;
	memcpy(&data, &this->cfgReg.gycfg, 1);
	this->write(GYRO_CONFIG, data);
}

MPU6050_FullscaleRange MPU6050::getFullScaleGyroRange() {
	if ((this->initValid & 0x02) != 0x02) this->updateConfigRegister();
	return (MPU6050_FullscaleRange) this->cfgReg.gycfg.fullscaleRange;
}

void MPU6050::setClockSource(MPU6050_ClockSource clksource) {
	uint8_t data;
	this->cfgReg.pwr1.clkSelect = clksource;
	memcpy(&data, &this->cfgReg.pwr1, 1);
	this->write(PWR_MGMT_1, data);
}

MPU6050_ClockSource MPU6050::getClockSource() {
	if ((this->initValid & 0x02) != 0x02) this->updateConfigRegister();
	return (MPU6050_ClockSource) this->cfgReg.pwr1.clkSelect;
}

void MPU6050::setSampleRateDivider(uint8_t div) {
	this->write(SMPLRT_DIV, div);
}

uint8_t MPU6050::getSampleRateDivider() {
	return this->read(SMPLRT_DIV);
}

void MPU6050::setLowPassFilterBandwidth(MPU6050_DigitalLowPassFilterBandwidth bw) {
	uint8_t data;
	this->cfgReg.cfg.digitalLowPassFilter = bw;
	memcpy(&data, &this->cfgReg.cfg, 1);
	this->write(CONFIG, data);
}

MPU6050_DigitalLowPassFilterBandwidth MPU6050::getLowPassFilterBandwidth() {
	if ((this->initValid & 0x02) != 0x02) this->updateConfigRegister();
	return (MPU6050_DigitalLowPassFilterBandwidth) this->cfgReg.cfg.digitalLowPassFilter;
}

void MPU6050::setConfigInterruptPin(bool level, bool driverType) {
	uint8_t data;
	this->cfgReg.intPinCfg.driveType = driverType;
	this->cfgReg.intPinCfg.level = level;
	memcpy(&data, &this->cfgReg.intPinCfg, 1);
	this->write(INT_PIN_CFG, data);
}

void MPU6050::setDisableTemperature(bool disable) {
	uint8_t data;
	if (disable)
		this->cfgReg.pwr1.temperatureDisable = true;
	else
		this->cfgReg.pwr1.temperatureDisable = false;

	memcpy(&data, &this->cfgReg.pwr1, 1);
	this->write(PWR_MGMT_1, data);
}

bool MPU6050::getDisableTemperature() {
	if ((this->initValid & 0x02) != 0x02) this->updateConfigRegister();
	return this->cfgReg.pwr1.temperatureDisable;
}

// Methods for reading data registers ****************************************************************************************************
void swapByte(uint8_t *byte1, uint8_t *byte2) {
	*byte1 += *byte2;
	*byte2 = *byte1 - *byte2;
	*byte1 -= *byte2;
}
void MPU6050::getRaw6Axis() {
	uint8_t temp[14] = { 0 };
	this->readBurst(ACCEL_XOUT_H, temp, sizeof(temp));
	for (uint8_t i = 0; i < sizeof(temp); i += 2) {
		swapByte(temp + i, temp + i + 1);
	}
	memcpy(&this->rawAxis.ax, temp, 6);
	memcpy(&this->rawAxis.gx, temp + 8, 6);
}

uint16_t MPU6050::getCountFIFO() {
	return this->readUnsignHalfWord(FIFO_COUNTH);
}

float MPU6050::getTemperature() {
	return ((float) this->readSignHalfWord(TEMP_OUT_H) / 340.0 + 36.53);
}

int16_t MPU6050::getRawAxis(AccelGyro ag, Axis axis) {
	uint8_t offset = 0;
	uint8_t gyroOffset = 8;
	uint8_t reg = ACCEL_XOUT_H;
	int16_t *pRawAxis = &this->rawAxis.ax;
	/* each of axis address take 2 bytes, first assign target addr is ACCEL_XOUT_H(0x3b)
	 * if axis = AXIS_Y = 1 then current register address need to offset 2 bytes, which is reg = ACCEL_OUT_H + 2
	 * the same as AXIS_Z = 2 -> reg = ACCEL_OUT_H + 4;
	 */
	offset += axis * 2;
	if (ag == GYRO) {
		offset += gyroOffset;
		reg += offset;
		*(pRawAxis + (offset - 2) / 2) = this->readSignHalfWord((MPU6050_RegisterAddress) reg); // (offset - 2) because Temperature register
	}
	else {
		reg += offset;
		*(pRawAxis + offset / 2) = this->readSignHalfWord((MPU6050_RegisterAddress) reg);
	}

	return this->readSignHalfWord((MPU6050_RegisterAddress) reg);
}

// Base methods to control read and write registers **************************************************************************************

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

void MPU6050::updateConfigRegister() {
	uint8_t data;
	data = this->read(PWR_MGMT_1);
	memcpy(&this->cfgReg.pwr1, &data, 1);
	data = this->read(GYRO_CONFIG);
	memcpy(&this->cfgReg.gycfg, &data, 1);
	data = this->read(INT_PIN_CFG);
	memcpy(&this->cfgReg.intPinCfg, &data, 1);
	data = this->read(CONFIG);
	memcpy(&this->cfgReg.cfg, &data, 1);
	this->initValid |= 0x02;
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
	if ((this->initValid & 0x01) != 0x01) return HAL_ERROR;
	if ((this->initValid & 0x02) != 0x02) this->updateConfigRegister();
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

uint16_t MPU6050::readUnsignHalfWord(MPU6050_RegisterAddress reg) {
	while (!this->initValid);
	uint8_t data[2];
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, data, 2, HAL_MAX_DELAY);
	return data[0] << 8 | data[1];
}

int16_t MPU6050::readSignHalfWord(MPU6050_RegisterAddress reg) {
	while (!this->initValid);
	uint8_t data[2];
	HAL_I2C_Master_Transmit(this->i2c, this->address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->i2c, this->address << 1, data, 2, HAL_MAX_DELAY);
	return data[0] << 8 | data[1];
}
