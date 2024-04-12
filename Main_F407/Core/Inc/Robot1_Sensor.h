/*
 * Robot1_Sensor.h
 *
 *  Created on: Apr 12, 2024
 *      Author: SpiritBoi
 */
#include "main.h"

typedef enum SensorError {
	SENSOR_ERR_SENSOR_NOT_DEFINE,
	SENSOR_ERR_REGISTER_PIN_NOT_MATCH_SENSOR_PORT,
	SENSOR_ERR_MAX_COUNT,
} SensorError;

typedef enum Robot1Sensor {
	RB1_SENSOR_ARM_LEFT = 1,
	RB1_SENSOR_ARM_RIGHT,
	RB1_SENSOR_COLLECT_BALL_LEFT,
	RB1_SENSOR_COLLECT_BALL_RIGHT,
} Robot1Sensor;

typedef struct Sensor_t {
	GPIO_TypeDef *gpioPort;
	uint16_t sensorPin;
	Robot1Sensor rb1sensor;
} Sensor_t;

void RB1_SensorRegisterPin(GPIO_TypeDef *gpioPort, uint16_t sensorPin, Robot1Sensor rb1SensorName);
void RB1_SensorTriggerHandle();
void RB1_WaitSensorInInterrupt(uint16_t sensorPin);
