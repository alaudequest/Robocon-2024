/*
 * Robot1_Sensor.c
 *
 *  Created on: Apr 12, 2024
 *      Author: SpiritBoi
 */

#include "Robot1_Sensor.h"
#include "stdbool.h"

#define SENSOR_DELAY_READ_MS 20
#define CONTINOUS_READ_NUM 3000

Sensor_t sensorArray[8] = { 0 };
uint8_t sensorRegisterCount = 1;
uint16_t currentSensorTrigger = 0;
Robot1Sensor rb1DetectSensor = 0;
uint8_t indexOfSensorTrigger = 0;
uint32_t delayTick = 0;

// dùng để lưu thứ tự đăng ký sensor tương ứng với loại cảm biến trong enum Robot1Sensor
// giúp giải quyết việc dùng vòng lặp for để quét sensorArray và tìm enum được request bởi người dùng
uint8_t keySensorEnum_ValueIndexOfSensorArray[4] = { 0 };

static void sensorErrorHandler(SensorError sErr);

/**
 * @fn void RB1_WaitSensorInInterrupt(uint16_t)
 * @brief hàm này phải được gọi trong void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *
 * @param sensorPin là chân ngắt từ hàm HAL_GPIO_EXTI_Callback truyền vào
 */
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

/**
 * @fn bool RB1_PollingReadSensor(Robot1Sensor)
 * @brief Sử dụng khi không dùng ngắt cảm biến
 * (có thể trong trường hợp nhiễu không đọc được ngắt như chạy động cơ)
 *
 * @param rb1SensorName đây là enum của Robot1Sensor chỉ ra loại cảm biến đang muốn đọc
 * @return
 */
bool RB1_PollingReadSensor(Robot1Sensor rb1SensorName)
{
	// Lấy thông tin cảm biến ở phần tử được mapping với enum Robot1Sensor
	Sensor_t sensor = sensorArray[keySensorEnum_ValueIndexOfSensorArray[rb1SensorName]];
	return HAL_GPIO_ReadPin(sensor.sensorPort, sensor.sensorPin);
}

/**
 * @fn void RB1_SensorTriggerHandle()
 * @brief Hàm này dùng chung với RB1_WaitSensorInInterrupt
 * Gọi hàm này trong loop while(1) của task hoặc int main() để xử lý khi phát hiện ngắt cảm biến
 *
 */
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
		// sau khi nhận dạng cảm biến, gọi tới hàm xử lý tương ứng do người dùng đăng ký
		if (sensorArray[indexOfSensorTrigger].sensorCallBack != NULL)
			sensorArray[indexOfSensorTrigger].sensorCallBack();
	}
	// Reset lại quá trình đọc cảm biến và cho phép ngắt
	delayTick = 0;
	indexOfSensorTrigger = 0;
	currentSensorTrigger = 0;
}

/**
 * @fn void RB1_SensorRegisterPin(GPIO_TypeDef*, uint16_t, Robot1Sensor)
 * @brief Đăng ký sử dụng cảm biến, lưu ý chân được cấu hình trên board phải khớp với phần cứng sử dụng
 * Ví dụ các chân cảm biến từ PA0 đến PA5 của board sử dụng dùng cho đọc cảm biến
 * thì không thể dùng chân PB0 không phục vụ mục đích đọc cảm biến để đăng ký
 * Đặc biệt khi dùng line ngắt (ví dụ PA0 trùng line ngắt với PB0) thì có thể chân ngắt cảm biến
 * bị trùng line ngắt với chân ngắt port khác trên board
 * @param sensorPort
 * @param sensorPin
 * @param rb1SensorName
 */
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
	//Cho biết loại cảm biến hiện tại đang được đăng ký ở phần tử sensorRegisterCount - 1 trong mảng sensorArray
	keySensorEnum_ValueIndexOfSensorArray[rb1SensorName] = sensorRegisterCount - 1;
	sensorRegisterCount++;
}

/**
 * @fn void RB1_RegisterSensorCallBack(void(*)(void), Robot1Sensor)
 * @brief Khi có sự kiện ngắt trên cảm biến thì sẽ gọi tới hàm do người dùng tự thiết kế
 * Có thể không sử dụng
 * @param pSensorCallback con trỏ hàm trỏ tới hàm do người dùng truyền vào
 * @param rb1SensorName
 */
void RB1_RegisterSensorCallBack(void (*pSensorCallback)(void), Robot1Sensor rb1SensorName)
{
	sensorArray[keySensorEnum_ValueIndexOfSensorArray[rb1SensorName]].sensorCallBack = pSensorCallback;
}

Sensor_t RB1_GetSensor(Robot1Sensor rb1SensorName)
{
	// Trả về thông tin cảm biến ở phần tử được mapping với enum Robot1Sensor
	return sensorArray[keySensorEnum_ValueIndexOfSensorArray[rb1SensorName]];
}

static void sensorErrorHandler(SensorError sErr)
{
	while (1);
}
