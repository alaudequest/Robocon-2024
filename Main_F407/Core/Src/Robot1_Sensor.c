/*
 * Robot1_Sensor.c
 *
 *  Created on: Apr 12, 2024
 *      Author: SpiritBoi
 */

#include "Robot1_Sensor.h"

#define SENSOR_DELAY_READ_MS 20
#define CONTINOUS_READ_NUM 3000

Sensor_t sensorArray[8] = { 0 };
uint8_t sensorRegisterCount = 1;
uint16_t currentSensorTrigger = 0;
Robot1Sensor rb1DetectSensor = 0;
uint8_t indexOfSensorTrigger = 0;
uint32_t delayTick = 0;
static void sensorErrorHandler(SensorError sErr);

void RB1_WaitSensorInInterrupt(uint16_t sensorPin)
{
	//Nếu đang trong quá trình xử lý sensor thì không nhận tín hiệu sensor mới
	// dùng để chống nhiễu nhảy vào ngắt ngoài gọi hàm liên tục
	if (currentSensorTrigger != 0)
		return;
	//Kiểm tra chân ngắt có thuộc các chân đã đăng ký trong mảng sensorArray
	for (uint8_t i = 0; i < 8; i++) {
		if (sensorArray[i].sensorPort == NULL)
			continue;
		if (sensorArray[i].sensorPin == sensorPin) {
			// Lưu thứ tự sensor kích trong mảng sensorArray để khỏi chạy lại for trong RB1_SensorTriggerHandle
			indexOfSensorTrigger = i;
			// Gán vào tạm biến để thoát khỏi hàm ngắt và xử lý trong loop
			// Không tắt line ngắt __NVIC_DisableIRQ(IRQn) vì ảnh hưởng tới ngắt ngoài encoder của cơ cấu rulo bắn
			currentSensorTrigger = sensorPin;
			delayTick = HAL_GetTick();			//bắt đầu delay
			return;
		}
	}
}

// Gọi hàm này trong vòng lặp xử lý bên main.c
void RB1_SensorTriggerHandle()
{
	//Nếu không có cảm biến thì return
	if (currentSensorTrigger == 0)
		return;
	// delay 20ms
	if (HAL_GetTick() - delayTick < SENSOR_DELAY_READ_MS)
		return;

	//Đọc liên tục, số lần đọc càng nhỏ thì càng ít ảnh hưởng tới chương trình,
	//nếu có nhiễu khiến reset số đếm thì tín hiệu đọc không đáng tin cậy
	uint16_t readCount = 0;
	for (uint16_t i = 0; i < CONTINOUS_READ_NUM; i++) {
		if (HAL_GPIO_ReadPin(sensorArray[indexOfSensorTrigger].sensorPort, sensorArray[indexOfSensorTrigger].sensorPin)) {
			readCount++;
		}
		else
			readCount = 0;
	}

	// Giả sử cho phép có gai nhiễu ở đầu quá trình đọc liên tục thì số đếm sẽ nhỏ hơn định mức một khoảng cho phép
	if (readCount > CONTINOUS_READ_NUM - 100) {
		rb1DetectSensor = sensorArray[indexOfSensorTrigger].rb1sensor;
		// sau khi nhận dạng cảm biến, gọi tới hàm xử lý tương ứng
		sensorArray[indexOfSensorTrigger].sensorCallBack();
	}
	// Reset lại quá trình đọc cảm biến và cho phép ngắt
	delayTick = 0;
	indexOfSensorTrigger = 0;
	currentSensorTrigger = 0;
}

void RB1_SensorRegisterPin(GPIO_TypeDef *sensorPort, uint16_t sensorPin, Robot1Sensor rb1SensorName)
{
	// Nếu port và pin không khớp từ PE7 tới PE14 (các chân đọc sensor) thì báo lỗi
	if (sensorPort != GPIOE || sensorPin < GPIO_PIN_7 || sensorPin > GPIO_PIN_14)
		sensorErrorHandler(SENSOR_ERR_REGISTER_PIN_NOT_MATCH_SENSOR_PORT);
	if (sensorRegisterCount > 8)
		sensorErrorHandler(SENSOR_ERR_MAX_COUNT);
	if (rb1SensorName < RB1_SENSOR_ARM_LEFT
			|| rb1SensorName > RB1_SENSOR_COLLECT_BALL_RIGHT) {
		sensorErrorHandler(SENSOR_ERR_SENSOR_NOT_DEFINE);
	}
	sensorArray[sensorRegisterCount - 1].sensorPort = sensorPort;
	sensorArray[sensorRegisterCount - 1].sensorPin = sensorPin;
	sensorArray[sensorRegisterCount - 1].rb1sensor = rb1SensorName;
	sensorRegisterCount++;
}

void RB1_RegisterSensorCallBack(void (*pSensorCallback)(void), Robot1Sensor rb1SensorName)
{
	for (uint8_t i = 0; i < 8; i++) {
		if (sensorArray[i].rb1sensor == rb1SensorName) {
			sensorArray[i].sensorCallBack = pSensorCallback;
			return;
		}
	}
}

Sensor_t RB1_GetSensor(Robot1Sensor rb1SensorName)
{
	for (uint8_t i = 0; i < 8; i++) {
		if (sensorArray[i].rb1sensor == rb1SensorName)
			return sensorArray[i];
	}
}

static void sensorErrorHandler(SensorError sErr)
{
	while (1);
}
