/*
 * CAN_Control.h
 *
 *  Created on: Sep 12, 2023
 *      Author: SpiritBoi
 */

#ifndef INC_CAN_CONTROL_H_
#define INC_CAN_CONTROL_H_
#include "main.h"
#include "stdbool.h"
#include "Flag.h"

#define CAN_DEVICE_POS 8

/*
 * Lưu ý khi chọn các giá trị enum cho filter mask thì không được để trùng bit lẫn nhau giữa các lệnh
 * Ví dụ enum có giá trị 3 = 0011 nghĩa là 2 bit 0 và 1 đã trùng nhau, enum có giá trị 6 = 0110 nghĩa là trùng bit 1 và bit 2
 * Vậy các enum không trùng là 1,2,4,8,16
 */
typedef enum CAN_MODE_ID{
	CANCTRL_MODE_START,
	CANCTRL_MODE_SET_HOME = 1, // Lệnh này dùng filter mask cho F4 (1 << 5), không được phép trùng bit các lệnh mask khác
	CANCTRL_MODE_TEST, // enum = 2 này chỉ có thể dùng filter list, vì mask trùng bit với CAN_RTR_REMOTE
	CANCTRL_MODE_MOTOR_SPEED_ANGLE,
	CANCTRL_MODE_ROBOT_ERROR = 4, // Lệnh này dùng filter mask cho F4 (4 << 5), không được phép trùng bit các lệnh mask khác
	CANCTRL_MODE_LED_BLUE,
	CANCTRL_MODE_MOTOR_BLDC_BRAKE,
	CANCTRL_MODE_PID_DC_SPEED,
	CANCTRL_MODE_PID_DC_ANGLE,
	CANCTRL_MODE_PID_BLDC_SPEED,
	CANCTRL_MODE_PID_BLDC_BREAKPROTECTION,
	CANCTRL_MODE_UNTANGLE_WIRE,
	CANCTRL_MODE_END,
}CAN_MODE_ID;

typedef enum CAN_DEVICE_ID{
	CANCTRL_DEVICE_MOTOR_CONTROLLER_1 = 1,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_2,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_3,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_4,
	CANCTRL_DEVICE_MAIN,
	CANCTRL_DEVICE_ACTUATOR_1,
	CANCTRL_DEVICE_ACTUATOR_2,
	CANCTRL_DEVICE_ACTUATOR_3,
}CAN_DEVICE_ID;
// 000  0000 0000
//{dev} {  } {mode}
//001 0000 0100 -> 0x104
// 0x001 ->  0000 0000 0001
// 0x002  -> 0000 0000 0010


CAN_RxHeaderTypeDef canctrl_GetRxHeader();
uint32_t canctrl_GetEvent();
void canctrl_SetTargetDevice(CAN_DEVICE_ID dev);
void canctrl_SetDLC(uint8_t DLC);
void canctrl_RTR_SetToData();
void canctrl_RTR_SetToRemote();
void canctrl_SetFlag(CAN_MODE_ID flag);
void canctrl_ClearFlag(CAN_MODE_ID flag);
bool canctrl_CheckFlag(CAN_MODE_ID flag);
HAL_StatusTypeDef canctrl_RTR_TxRequest(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID, CAN_MODE_ID modeID);
CAN_MODE_ID canctrl_RxHeaderGetModeID();
CAN_MODE_ID canctrl_Receive_2(CAN_HandleTypeDef *can, uint32_t FIFO);
HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t RTR);
HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can, CAN_DEVICE_ID targetID);
HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO);
HAL_StatusTypeDef canctrl_SetID(uint32_t ID);
HAL_StatusTypeDef canctrl_PutMessage(void* data,size_t dataSize);
HAL_StatusTypeDef canctrl_GetMessage(void *data, size_t sizeOfDataType);
HAL_StatusTypeDef canctrl_FilCfg(CAN_HandleTypeDef *can, uint32_t filterID, uint32_t filBank, uint32_t FIFO);
HAL_StatusTypeDef canctrl_Init(CAN_HandleTypeDef *can);

HAL_StatusTypeDef canctrl_SendMultipleMessages(CAN_HandleTypeDef *can,
											CAN_DEVICE_ID targetID,
											void *data,
											size_t sizeOfDataType);

HAL_StatusTypeDef canctrl_GetMultipleMessages(void *data, size_t sizeOfDataType);

HAL_StatusTypeDef canctrl_Filter_List16(CAN_HandleTypeDef *can,
												uint16_t ID1,
												uint16_t ID2,
												uint16_t ID3,
												uint16_t ID4,
												uint32_t filBank,
												uint32_t FIFO);
HAL_StatusTypeDef canctrl_Filter_Mask16(CAN_HandleTypeDef *can,
												uint16_t highID,
												uint16_t lowID,
												uint16_t maskHigh,
												uint16_t maskLow,
												uint32_t filBank,
												uint32_t FIFO);
#endif /* INC_CAN_CONTROL_H_ */
