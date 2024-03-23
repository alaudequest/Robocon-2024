/*
 * Sensor.h
 *
 *  Created on: Mar 21, 2024
 *      Author: KHOA
 */

typedef void (*pSensorTrigger)(void*);



typedef enum SensorName {
	SENSOR_LASER,
	SENSOR_FORKLIFT,
	SENSOR_SILO,
} SensorName;

typedef struct SensorTriggerProcess {
	SensorName nextSensor;
} SensorTriggerProcess;
