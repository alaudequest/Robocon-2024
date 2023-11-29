/*
 * MPU6050_RegisterDef.h
 *
 *  Created on: Nov 28, 2023
 *      Author: KHOA
 */

#ifndef INC_MPU6050_REGISTERDEF_H_
#define INC_MPU6050_REGISTERDEF_H_

typedef enum MPU6050_RegisterAddress {
	XG_OFFS_TC = 0x00, //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
	YG_OFFS_TC = 0x01, //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
	ZG_OFFS_TC = 0x02, //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
	X_FINE_GAIN = 0x03, //[7:0] X_FINE_GAIN
	Y_FINE_GAIN = 0x04, //[7:0] Y_FINE_GAIN
	Z_FINE_GAIN = 0x05, //[7:0] Z_FINE_GAIN
	XA_OFFS_H = 0x06, //[15:0] XA_OFFS
	XA_OFFS_L_TC = 0x07,
	YA_OFFS_H = 0x08, //[15:0] YA_OFFS
	YA_OFFS_L_TC = 0x09,
	ZA_OFFS_H = 0x0A, //[15:0] ZA_OFFS
	ZA_OFFS_L_TC = 0x0B,
	SELF_TEST_X = 0x0D, //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
	SELF_TEST_Y = 0x0E, //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
	SELF_TEST_Z = 0x0F, //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
	SELF_TEST_A = 0x10, //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
	XG_OFFS_USRH = 0x13, //[15:0] XG_OFFS_USR
	XG_OFFS_USRL = 0x14,
	YG_OFFS_USRH = 0x15, //[15:0] YG_OFFS_USR
	YG_OFFS_USRL = 0x16,
	ZG_OFFS_USRH = 0x17, //[15:0] ZG_OFFS_USR
	ZG_OFFS_USRL = 0x18,
	SMPLRT_DIV = 0x19,
	CONFIG = 0x1A,
	GYRO_CONFIG = 0x1B,
	ACCEL_CONFIG = 0x1C,
	FF_THR = 0x1D,
	FF_DUR = 0x1E,
	MOT_THR = 0x1F,
	MOT_DUR = 0x20,
	ZRMOT_THR = 0x21,
	ZRMOT_DUR = 0x22,
	FIFO_EN = 0x23,
	I2C_MST_CTRL = 0x24,
	I2C_SLV0_ADDR = 0x25,
	I2C_SLV0_REG = 0x26,
	I2C_SLV0_CTRL = 0x27,
	I2C_SLV1_ADDR = 0x28,
	I2C_SLV1_REG = 0x29,
	I2C_SLV1_CTRL = 0x2A,
	I2C_SLV2_ADDR = 0x2B,
	I2C_SLV2_REG = 0x2C,
	I2C_SLV2_CTRL = 0x2D,
	I2C_SLV3_ADDR = 0x2E,
	I2C_SLV3_REG = 0x2F,
	I2C_SLV3_CTRL = 0x30,
	I2C_SLV4_ADDR = 0x31,
	I2C_SLV4_REG = 0x32,
	I2C_SLV4_DO = 0x33,
	I2C_SLV4_CTRL = 0x34,
	I2C_SLV4_DI = 0x35,
	I2C_MST_STATUS = 0x36,
	INT_PIN_CFG = 0x37,
	INT_ENABLE = 0x38,
	DMP_INT_STATUS = 0x39,
	INT_STATUS = 0x3A,
	ACCEL_XOUT_H = 0x3B,
	ACCEL_XOUT_L = 0x3C,
	ACCEL_YOUT_H = 0x3D,
	ACCEL_YOUT_L = 0x3E,
	ACCEL_ZOUT_H = 0x3F,
	ACCEL_ZOUT_L = 0x40,
	TEMP_OUT_H = 0x41,
	TEMP_OUT_L = 0x42,
	GYRO_XOUT_H = 0x43,
	GYRO_XOUT_L = 0x44,
	GYRO_YOUT_H = 0x45,
	GYRO_YOUT_L = 0x46,
	GYRO_ZOUT_H = 0x47,
	GYRO_ZOUT_L = 0x48,
	EXT_SENS_DATA_00 = 0x49,
	EXT_SENS_DATA_01 = 0x4A,
	EXT_SENS_DATA_02 = 0x4B,
	EXT_SENS_DATA_03 = 0x4C,
	EXT_SENS_DATA_04 = 0x4D,
	EXT_SENS_DATA_05 = 0x4E,
	EXT_SENS_DATA_06 = 0x4F,
	EXT_SENS_DATA_07 = 0x50,
	EXT_SENS_DATA_08 = 0x51,
	EXT_SENS_DATA_09 = 0x52,
	EXT_SENS_DATA_10 = 0x53,
	EXT_SENS_DATA_11 = 0x54,
	EXT_SENS_DATA_12 = 0x55,
	EXT_SENS_DATA_13 = 0x56,
	EXT_SENS_DATA_14 = 0x57,
	EXT_SENS_DATA_15 = 0x58,
	EXT_SENS_DATA_16 = 0x59,
	EXT_SENS_DATA_17 = 0x5A,
	EXT_SENS_DATA_18 = 0x5B,
	EXT_SENS_DATA_19 = 0x5C,
	EXT_SENS_DATA_20 = 0x5D,
	EXT_SENS_DATA_21 = 0x5E,
	EXT_SENS_DATA_22 = 0x5F,
	EXT_SENS_DATA_23 = 0x60,
	MOT_DETECT_STATUS = 0x61,
	I2C_SLV0_DO = 0x63,
	I2C_SLV1_DO = 0x64,
	I2C_SLV2_DO = 0x65,
	I2C_SLV3_DO = 0x66,
	I2C_MST_DELAY_CTRL = 0x67,
	SIGNAL_PATH_RESET = 0x68,
	MOT_DETECT_CTRL = 0x69,
	USER_CTRL = 0x6A,
	PWR_MGMT_1 = 0x6B,
	PWR_MGMT_2 = 0x6C,
	BANK_SEL = 0x6D,
	MEM_START_ADDR = 0x6E,
	MEM_R_W = 0x6F,
	DMP_CFG_1 = 0x70,
	DMP_CFG_2 = 0x71,
	FIFO_COUNTH = 0x72,
	FIFO_COUNTL = 0x73,
	FIFO_R_W = 0x74,
	WHO_AM_I = 0x75,
} MPU6050_RegisterAddress;

/* INTERRUPT PIN CONFIG--------------------------------------------------------------------------------*/

#define INTLEVEL_ACTIVEHIGH 0x00
#define INTLEVEL_ACTIVELOW  0x01
#define INTDRV_PUSHPULL     0x00
#define INTDRV_OPENDRAIN    0x01
#define INTLATCH_50USPULSE  0x00
#define INTLATCH_WAITCLEAR  0x01
#define INTCLEAR_STATUSREAD 0x00
#define INTCLEAR_ANYREAD    0x01
typedef struct MPU6050_InterruptPinConfig {
		uint8_t reserve :1;
		uint8_t i2cBypassEnable :1;
		uint8_t fsyncIntEnable :1;
		uint8_t fsyncIntLevel :1;
		uint8_t intRdClear :1;
		uint8_t latchInt :1;
		uint8_t driveType :1;
		uint8_t level :1;
} MPU6050_InterruptPinConfig;

/* POWER MANAGEMENT 1--------------------------------------------------------------------------------*/

/*
 * CLKSEL Clock Source
 * 0 Internal 8MHz oscillator
 * 1 PLL with X axis gyroscope reference
 * 2 PLL with Y axis gyroscope reference
 * 3 PLL with Z axis gyroscope reference
 * 4 PLL with external 32.768kHz reference
 * 5 PLL with external 19.2MHz reference
 * 6 Reserved
 * 7 Stops the clock and keeps the timing generator in reset
 *
 */

typedef enum MPU6050_ClockSource {
	CLOCK_INTERNAL = 0x00,
	CLOCK_PLL_XGYRO = 0x01,
	CLOCK_PLL_YGYRO = 0x02,
	CLOCK_PLL_ZGYRO = 0x03,
	CLOCK_PLL_EXT32K = 0x04,
	CLOCK_PLL_EXT19M = 0x05,
	CLOCK_KEEP_RESET = 0x07,
} MPU6050_ClockSource;

typedef struct MPU6050_PowerManagement1 {
		uint8_t clkSelect :3;
		uint8_t temperatureDisable :1;
		uint8_t reserve :1;
		uint8_t cycle :1;
		uint8_t sleep :1;
		uint8_t devReset :1;
} MPU6050_PowerManagement1;

/* CONFIGURATION--------------------------------------------------------------------------------*/

/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */

typedef enum MPU6050_ExternalSync {
	DISABLED = 0x0,
	EXT_TEMP_OUT_L = 0x1,
	EXT_GYRO_XOUT_L = 0x2,
	EXT_GYRO_YOUT_L = 0x3,
	EXT_GYRO_ZOUT_L = 0x4,
	EXT_ACCEL_XOUT_L = 0x5,
	EXT_ACCEL_YOUT_L = 0x6,
	EXT_ACCEL_ZOUT_L = 0x7,
} MPU6050_ExternalSync;

typedef enum MPU6050_DigitalLowPassFilterBandwidth {
	BW_256 = 0x00,
	BW_188 = 0x01,
	BW_98 = 0x02,
	BW_42 = 0x03,
	BW_20 = 0x04,
	BW_10 = 0x05,
	BW_5 = 0x06,
} MPU6050_DigitalLowPassFilterBandwidth;

typedef struct MPU6050_Configuration {
		uint8_t reserve :2;
		uint8_t externalSync :3;
		uint8_t digitalLowPassFilter :3;
} MPU6050_Configuration;

/* INTERRUPT ENABLE--------------------------------------------------------------------------------*/
typedef struct MPU6050_InterruptEnable {
		uint8_t reserve :3;
		uint8_t OverflowFIFO :1;
		uint8_t MasterInterrupt :1;
		uint8_t reserve2 :2;
		uint8_t DataReady :1;
} MPU6050_InterruptEnable;

/* SEFT TEST--------------------------------------------------------------------------------*/

typedef struct MPU6050_SeftTest {
		struct axisX {
				uint8_t xA_High3Bit :3; //bit 2 to bit 4
				uint8_t xG :5;
		} axisX;
		struct axisY {
				uint8_t yA_High3Bit :3; //bit 2 to bit 4
				uint8_t yG :5;
		} axisY;
		struct axisZ {
				uint8_t zA_High3Bit :3; //bit 2 to bit 4
				uint8_t zG :5;
		} axisZ;
		struct axisLowBit {
				uint8_t reserve :2;
				uint8_t xA_Low2Bit :2; //bit 0 to bit 1
				uint8_t yA_Low2Bit :2; //bit 0 to bit 1
				uint8_t zA_Low2Bit :2; //bit 0 to bit 1
		} axisLowBit;
} MPU6050_SeftTest;

/* GYRO CONFIG--------------------------------------------------------------------------------*/
/*
 * FS_SEL Full Scale Range LSB Sensitivity
 * 0 ± 250 °/s 131 LSB/°/s
 * 1 ± 500 °/s 65.5 LSB/°/s
 * 2 ± 1000 °/s 32.8 LSB/°/s
 * 3 ± 2000 °/s 16.4 LSB/°/s
 */

typedef enum MPU6050_FullscaleRange {
	FS_250 = 0x00,
	FS_500 = 0x01,
	FS_1000 = 0x02,
	FS_2000 = 0x03,
} MPU6050_FullscaleRange;

typedef struct MPU6050_GyroConfig {
		uint8_t XG_ST :1;
		uint8_t YG_ST :1;
		uint8_t ZG_ST :1;
		uint8_t fullscaleRange :3;
		uint8_t reserve :3;
} MPU6050_GyroConfig;

#endif /* INC_MPU6050_REGISTERDEF_H_ */
