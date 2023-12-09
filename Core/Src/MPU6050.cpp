/*
 * MPU6050.cpp
 *
 *  Created on: Nov 28, 2023
 *      Author: KHOA
 */

#include "MPU6050.h"
#include "string.h"
// Methods for configuring config registers ****************************************************************************************************
void MPU6050::setFullScaleGyroRange(MPU6050_GyroFullscaleRange fs) {
	uint8_t data;
	cfgReg.gycfg.fullscaleRange = fs;
	memcpy(&data, &cfgReg.gycfg, 1);
	write(GYRO_CONFIG, data);
}

MPU6050_GyroFullscaleRange MPU6050::getFullScaleGyroRange() {
	if ((initValid & 0x02) != 0x02) updateConfigRegister();
	return (MPU6050_GyroFullscaleRange) cfgReg.gycfg.fullscaleRange;
}

void MPU6050::setFullScaleAccelRange(MPU6050_AccelFullscaleRange afs) {
	uint8_t data;
	cfgReg.acccfg.fullscaleRange = afs;
	memcpy(&data, &cfgReg.acccfg, 1);
	write(ACCEL_CONFIG, data);
}

MPU6050_AccelFullscaleRange MPU6050::getFullScaleAccelRange() {
	if ((initValid & 0x02) != 0x02) updateConfigRegister();
	return (MPU6050_AccelFullscaleRange) cfgReg.acccfg.fullscaleRange;
}

void MPU6050::setClockSource(MPU6050_ClockSource clksource) {
	uint8_t data;
	cfgReg.pwr1.clkSelect = clksource;
	memcpy(&data, &cfgReg.pwr1, 1);
	write(PWR_MGMT_1, data);
}

MPU6050_ClockSource MPU6050::getClockSource() {
	if ((initValid & 0x02) != 0x02) updateConfigRegister();
	return (MPU6050_ClockSource) cfgReg.pwr1.clkSelect;
}

void MPU6050::setSampleRateDivider(uint8_t div) {
	write(SMPLRT_DIV, div);
}

uint8_t MPU6050::getSampleRateDivider() {
	return read(SMPLRT_DIV);
}

void MPU6050::setLowPassFilterBandwidth(MPU6050_DigitalLowPassFilterBandwidth bw) {
	uint8_t data;
	cfgReg.cfg.digitalLowPassFilter = bw;
	memcpy(&data, &cfgReg.cfg, 1);
	write(CONFIG, data);
}

MPU6050_DigitalLowPassFilterBandwidth MPU6050::getLowPassFilterBandwidth() {
	if ((initValid & 0x02) != 0x02) updateConfigRegister();
	return (MPU6050_DigitalLowPassFilterBandwidth) cfgReg.cfg.digitalLowPassFilter;
}

void MPU6050::setConfigInterruptPin(bool level, bool driverType) {
	uint8_t data;
	cfgReg.intPinCfg.driveType = driverType;
	cfgReg.intPinCfg.level = level;
	memcpy(&data, &cfgReg.intPinCfg, 1);
	write(INT_PIN_CFG, data);
}

void MPU6050::setDisableTemperature(bool disable) {
	uint8_t data;
	cfgReg.pwr1.temperatureDisable = disable;
	memcpy(&data, &cfgReg.pwr1, 1);
	write(PWR_MGMT_1, data);
}

bool MPU6050::getDisableTemperature() {
	if ((initValid & 0x02) != 0x02) updateConfigRegister();
	return cfgReg.pwr1.temperatureDisable;
}

void MPU6050::fifoEnableTemperature(bool enable) {
	uint8_t data;
	cfgReg.fifoEn.temperature = enable;
	memcpy(&data, &cfgReg.fifoEn, 1);
	write(FIFO_EN, data);
}
void MPU6050::fifoEnableGyro(Axis axis, bool enable) {
	uint8_t data;
	switch (axis) {
		case AXIS_X:
			cfgReg.fifoEn.xg = enable;
		break;
		case AXIS_Y:
			cfgReg.fifoEn.yg = enable;
		break;
		case AXIS_Z:
			cfgReg.fifoEn.zg = enable;
		break;

	}
	memcpy(&data, &cfgReg.fifoEn, 1);
	write(FIFO_EN, data);
}
void MPU6050::fifoEnableAccel(bool enable) {
	uint8_t data;
	cfgReg.fifoEn.accel = enable;
	memcpy(&data, &cfgReg.fifoEn, 1);
	write(FIFO_EN, data);
}
void MPU6050::fifoEnable(bool enable) {
	uint8_t data;
	cfgReg.usrCtrl.fifoEnable = enable;
	memcpy(&data, &cfgReg.usrCtrl, 1);
	write(USER_CTRL, data);
}
HAL_StatusTypeDef MPU6050::fifoReset() {
	uint8_t data;
	// According to datasheet, should disable FIFO before reset
	cfgReg.usrCtrl.fifoEnable = 0;
	memcpy(&data, &cfgReg.usrCtrl, 1);
	write(USER_CTRL, data);

	cfgReg.usrCtrl.fifoReset = 1;
	memcpy(&data, &cfgReg.usrCtrl, 1);
	write(USER_CTRL, data);

	// Automatically set to 0 after triggering reset, check if FIFO_RESET is 0
	data = read(USER_CTRL);
	memcpy(&cfgReg.usrCtrl, &data, 1);
	if (!cfgReg.usrCtrl.fifoReset)
		return HAL_OK;
	else
		return HAL_ERROR;
}

void MPU6050::fifoSetOverflowInterrupt(bool enable) {
	uint8_t data;
	cfgReg.intCfgStatus.OverflowFIFO = enable;
	memcpy(&data, &cfgReg.intCfgStatus, 1);
	write(INT_ENABLE, data);
}
void MPU6050::fifoSetDataReadyInterrupt(bool enable) {
	uint8_t data;
	cfgReg.intCfgStatus.DataReady = enable;
	memcpy(&data, &cfgReg.intCfgStatus, 1);
	write(INT_ENABLE, data);
}

// Methods for reading data registers ****************************************************************************************************
void swapByte(uint8_t *byte1, uint8_t *byte2) {
	*byte1 += *byte2;
	*byte2 = *byte1 - *byte2;
	*byte1 -= *byte2;
}
rawDataAxis MPU6050::getRaw6Axis() {
	uint8_t temp[14] = { 0 };
	readBurst(ACCEL_XOUT_H, temp, sizeof(temp));
	for (uint8_t i = 0; i < sizeof(temp); i += 2) {
		swapByte(temp + i, temp + i + 1);
	}
	memcpy(&rawAxis.ax, temp, 6);
	memcpy(&rawAxis.gx, temp + 8, 6);
	return rawAxis;
}

uint16_t MPU6050::getCountFIFO() {
	return readUnsignHalfWord(FIFO_COUNTH);
}

float MPU6050::getTemperature() {
	return ((float) readSignHalfWord(TEMP_OUT_H) / 340.0 + 36.53);
}

int16_t MPU6050::getRawAxis(AccelGyro ag, Axis axis) {
	uint8_t offset = 0;
	uint8_t gyroOffset = 8;
	uint8_t reg = ACCEL_XOUT_H;
	int16_t *pRawAxis = &rawAxis.ax;
	/* each of axis address take 2 bytes, first assign target addr is ACCEL_XOUT_H(0x3b)
	 * if axis = AXIS_Y = 1 then current register address need to offset 2 bytes, which is reg = ACCEL_OUT_H + 2
	 * the same as AXIS_Z = 2 -> reg = ACCEL_OUT_H + 4;
	 */
	offset += axis * 2;
	if (ag == GYRO) {
		offset += gyroOffset;
		reg += offset;
		*(pRawAxis + (offset - 2) / 2) = readSignHalfWord((MPU6050_RegisterAddress) reg); // (offset - 2) because Temperature register
	}
	else {
		reg += offset;
		*(pRawAxis + offset / 2) = readSignHalfWord((MPU6050_RegisterAddress) reg);
	}

	return readSignHalfWord((MPU6050_RegisterAddress) reg);
}

float MPU6050::getGyro(Axis axis) {
	uint8_t temp;
	int16_t rawAxis;
	rawAxis = getRawAxis(GYRO, axis);
	//LSB Sensitivity divided by 2 for each FS_SEL + 1, so 1 << n is 2^n
	temp = 1 << (uint8_t) getFullScaleGyroRange();
	return ((float) rawAxis / (float) (131 / temp));
}

float MPU6050::getAccel(Axis axis) {
	uint8_t temp;
	int16_t rawAxis;
	rawAxis = getRawAxis(ACCEL, axis);
	//LSB Sensitivity divided by 2 for each AFS_SEL + 1, so 1 << n is 2^n
	temp = 1 << (uint8_t) getFullScaleAccelRange();
	return ((float) rawAxis / (float) (16384 / temp));
}

float MPU6050::getRoll() {
	return 0;
}
float MPU6050::getPitch() {
	return 0;
}
float MPU6050::getYaw() {
	float Ax = getAccel(AXIS_X) + OFFSET_AX;
	float Ay = getAccel(AXIS_Y) + OFFSET_AY;
	float Az = getAccel(AXIS_Z) + OFFSET_AZ;
	float yawRad = atan((sqrt(Ax * Ax + Ay * Ay)) / Az);
	return yawRad * 180 / 3.142;
}
bool MPU6050::isDataReady() {
	uint8_t data = read(INT_STATUS);
	if (data & 0x01)
		return true;
	else
		return false;

}
bool MPU6050::isOverflowFIFO() {
	uint8_t data = read(INT_STATUS);
	if (data & 0x10)
		return true;
	else
		return false;
}
// Base methods to control read and write registers **************************************************************************************

HAL_StatusTypeDef MPU6050::writeBurst(MPU6050_RegisterAddress reg, uint8_t *wdata, uint8_t writeTime) {
	if (!initValid) return HAL_ERROR;
	HAL_I2C_Master_Transmit(i2c, address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(i2c, address << 1, wdata, writeTime, HAL_MAX_DELAY);
	return HAL_OK;

}

HAL_StatusTypeDef MPU6050::readBurst(MPU6050_RegisterAddress reg, uint8_t *rdata, uint8_t readTime) {
	if (!initValid) return HAL_ERROR;
	HAL_I2C_Master_Transmit(i2c, address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(i2c, address << 1, rdata, readTime, HAL_MAX_DELAY);

	return HAL_OK;
}

void MPU6050::updateConfigRegister() {
	uint8_t data;
	data = read(PWR_MGMT_1);
	memcpy(&cfgReg.pwr1, &data, 1);
	data = read(GYRO_CONFIG);
	memcpy(&cfgReg.gycfg, &data, 1);
	data = read(ACCEL_CONFIG);
	memcpy(&cfgReg.acccfg, &data, 1);
	data = read(INT_PIN_CFG);
	memcpy(&cfgReg.intPinCfg, &data, 1);
	data = read(CONFIG);
	memcpy(&cfgReg.cfg, &data, 1);
	data = read(FIFO_EN);
	memcpy(&cfgReg.fifoEn, &data, 1);
	data = read(USER_CTRL);
	memcpy(&cfgReg.usrCtrl, &data, 1);
	initValid |= 0x02;
}

void MPU6050::init(I2C_HandleTypeDef *i2c, bool addressHigh) {
	this->i2c = i2c;
	if (addressHigh)
		address = MPU6050_ADDRESS_AD0_HIGH;
	else
		address = MPU6050_ADDRESS_AD0_LOW;
	initValid = 1;
	while (isReady() != HAL_OK);
//internal clock cannot read value, and clock from gyro is better
	setClockSource(CLOCK_PLL_XGYRO);
	setFullScaleGyroRange(FS_500);
	setFullScaleAccelRange(AFS_8g);
	setLowPassFilterBandwidth(BW_98);
	setDisableTemperature(true);
	setConfigInterruptPin(INTLEVEL_ACTIVELOW, INTDRV_PUSHPULL);
}

HAL_StatusTypeDef MPU6050::isReady() {
	if (!initValid) return HAL_ERROR;
	return HAL_I2C_IsDeviceReady(i2c, address << 1, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050::write(MPU6050_RegisterAddress reg, uint8_t data) {
	if ((initValid & 0x01) != 0x01) return HAL_ERROR;
	if ((initValid & 0x02) != 0x02) updateConfigRegister();
	uint8_t dataTemp[2] = { reg, data };
	return HAL_I2C_Master_Transmit(i2c, address << 1, dataTemp, 2, HAL_MAX_DELAY);

}

uint8_t MPU6050::read(MPU6050_RegisterAddress reg) {
	while (!initValid);
	uint8_t data[1];
	HAL_I2C_Master_Transmit(i2c, address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(i2c, address << 1, data, 1, HAL_MAX_DELAY);
	return data[0];
}

uint16_t MPU6050::readUnsignHalfWord(MPU6050_RegisterAddress reg) {
	while (!initValid);
	uint8_t data[2];
	HAL_I2C_Master_Transmit(i2c, address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(i2c, address << 1, data, 2, HAL_MAX_DELAY);
	return data[0] << 8 | data[1];
}

int16_t MPU6050::readSignHalfWord(MPU6050_RegisterAddress reg) {
	while (!initValid);
	uint8_t data[2];
	HAL_I2C_Master_Transmit(i2c, address << 1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(i2c, address << 1, data, 2, HAL_MAX_DELAY);
	return data[0] << 8 | data[1];
}
