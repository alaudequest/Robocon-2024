/*
 * Robot1_Sensor.c
 *
 *  Created on: Apr 12, 2024
 *      Author: SpiritBoi
 */

#include "Robot1_Sensor.h"
Sensor_t sensorArray[8] = { 0 };
uint8_t sensorRegisterCount = 1;
uint16_t currentSensorTrigger = 0;
Robot1Sensor rb1DetectSensor = 0;
static void sensorErrorHandler(SensorError sErr);

void RB1_WaitSensorInInterrupt(uint16_t sensorPin)
{
	//Nếu đang trong quá trình xử lý sensor thì không nhận tín hiệu sensor mới
	// dùng để chống nhiễu nhảy vào ngát ngoài gọi hàm liên tục
	if (currentSensorTrigger != 0)
		return;
	currentSensorTrigger = sensorPin;
}

// Gọi hàm này trong vòng lặp xử lý bên main.c
void RB1_SensorTriggerHandle()
{
	//Nếu không có cảm biến thì return
	if (currentSensorTrigger == 0)
		return;
	for (uint8_t i = 0; i < 8; i++) {
		if (sensorArray[i].sensorPin == currentSensorTrigger) {
			rb1DetectSensor = sensorArray[i].rb1sensor;
			break;
		}
	}
	currentSensorTrigger = 0;
}

void RB1_SensorRegisterPin(GPIO_TypeDef *gpioPort, uint16_t sensorPin, Robot1Sensor rb1SensorName)
{
	// Nếu port và pin không khớp từ PE7 tới PE14 (các chân đọc sensor) thì báo lỗi
	if (gpioPort != GPIOE || sensorPin < GPIO_PIN_7 || sensorPin > GPIO_PIN_14)
		sensorErrorHandler(SENSOR_ERR_REGISTER_PIN_NOT_MATCH_SENSOR_PORT);
	if (sensorRegisterCount > 8)
		sensorErrorHandler(SENSOR_ERR_MAX_COUNT);
	if (rb1SensorName < RB1_SENSOR_ARM_LEFT
			|| rb1SensorName > RB1_SENSOR_COLLECT_BALL_RIGHT) {
		sensorErrorHandler(SENSOR_ERR_SENSOR_NOT_DEFINE);
	}
	sensorArray[sensorRegisterCount - 1].gpioPort = gpioPort;
	sensorArray[sensorRegisterCount - 1].sensorPin = sensorPin;
	sensorArray[sensorRegisterCount - 1].rb1sensor = rb1SensorName;
	sensorRegisterCount++;
}

static void sensorErrorHandler(SensorError sErr)
{
	while (1);
}
