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

typedef struct ConfigRegister {
		MPU6050_PowerManagement1 pwr1;
		MPU6050_GyroConfig gycfg;
		MPU6050_Configuration cfg;
		MPU6050_InterruptPinConfig intPinCfg;
} ConfigRegister;

typedef enum Axis {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
} Axis;

typedef enum AccelGyro {
	ACCEL,
	GYRO,
} AccelGyro;

typedef struct rawDataAxis {
		int16_t ax;
		int16_t ay;
		int16_t az;
		int16_t gx;
		int16_t gy;
		int16_t gz;
} rawDataAxis;

class MPU6050 {
	private:
		I2C_HandleTypeDef *i2c;
		uint8_t address;
		uint8_t initValid;
		ConfigRegister cfgReg;
		rawDataAxis rawAxis;

	public:
		// base methods to reading and writing registers
		void init(I2C_HandleTypeDef *i2c, bool addressHigh);
		HAL_StatusTypeDef write(MPU6050_RegisterAddress reg, uint8_t data);
		uint8_t read(MPU6050_RegisterAddress reg);
		int16_t readSignHalfWord(MPU6050_RegisterAddress reg);
		uint16_t readUnsignHalfWord(MPU6050_RegisterAddress reg);
		HAL_StatusTypeDef writeBurst(MPU6050_RegisterAddress reg, uint8_t *wdata, uint8_t writeTime);
		HAL_StatusTypeDef readBurst(MPU6050_RegisterAddress reg, uint8_t *rdata, uint8_t readTime);
		HAL_StatusTypeDef isReady();

		// methods for reading data registers
		uint16_t getCountFIFO();
		float getTemperature();
		rawDataAxis getRaw6Axis();
		int16_t getRawAxis(AccelGyro ag, Axis axis);

		// methods for configuring Config registers
		void setSampleRateDivider(uint8_t div);
		uint8_t getSampleRateDivider();
		void setClockSource(MPU6050_ClockSource clksource);
		MPU6050_ClockSource getClockSource();
		void setLowPassFilterBandwidth(MPU6050_DigitalLowPassFilterBandwidth bw);
		MPU6050_DigitalLowPassFilterBandwidth getLowPassFilterBandwidth();
		void setFullScaleGyroRange(MPU6050_FullscaleRange fs);
		MPU6050_FullscaleRange getFullScaleGyroRange();
		void setDisableTemperature(bool disable);
		bool getDisableTemperature();
		void setConfigInterruptPin(bool level, bool driverType);
		void updateConfigRegister();

};

#endif

#endif /* INC_MPU6050_H_ */
