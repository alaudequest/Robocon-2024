/*
 * CAN_Control.h
 *
 *  Created on: Sep 12, 2023
 *      Author: KHOA
 */

#ifndef INC_CAN_CONTROL_H_
#define INC_CAN_CONTROL_H_
#include "main.h"
#include "stdbool.h"

#define CAN_EVT_CHECKFLAG(FlagBit) ((((canEvent) & (1 << FlagBit)) == (1 << FlagBit)) ? 1 : 0)
#define CAN_EVT_SETFLAG(FlagBit) ((canEvent) |= (1 << FlagBit))
#define CAN_EVT_CLEARFLAG(FlagBit) ((canEvent) &= ~(1 << FlagBit))
#define CAN_DEVICE_POS 8
typedef enum CAN_MODE_ID{
	CANCTRL_MODE_START,
	CANCTRL_MODE_LED_STATE,
	CANCTRL_MODE_LED_BLINK,
	CANCTRL_MODE_SHOOT,
	CANCTRL_MODE_ENCODER,
	CANCTRL_MODE_SET_HOME,
	CANCTRL_MODE_MOTOR_SPEED_ANGLE,
	CANCTRL_MODE_MOTOR_BLDC_BRAKE,
	CANCTRL_MODE_PID_DC_SPEED,
	CANCTRL_MODE_PID_DC_ANGLE,
	CANCTRL_MODE_PID_BLDC,
	CANCTRL_MODE_END,
}CAN_MODE_ID;

typedef enum CAN_DEVICE_ID{
	CANCTRL_DEVICE_MAIN,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_1,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_2,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_3,
	CANCTRL_DEVICE_MOTOR_CONTROLLER_4,
	CANCTRL_DEVICE_ACTUATOR_1,
	CANCTRL_DEVICE_ACTUATOR_2,
	CANCTRL_DEVICE_ACTUATOR_3,
}CAN_DEVICE_ID;

typedef union fByte{
	float floatData;
	uint8_t byteData[4];
}fByte;


typedef union iByte{
	uint64_t intData;
	uint8_t byteData[8];
}iByte;


CAN_RxHeaderTypeDef canctrl_GetRxHeader();
float canctrl_GetFloatNum();
uint64_t canctrl_GetIntNum();
uint32_t canctrl_GetDLC();
uint32_t canctrl_GetID();
uint32_t canctrl_GetEvent();
void canctrl_SetTargetDevice(CAN_DEVICE_ID dev);
void canctrl_SetDLC(uint8_t DLC);
void canctrl_RTR_SetToData();
void canctrl_RTR_SetToRemote();
void canctrl_SetFlag(CAN_MODE_ID flag);
void canctrl_ClearFlag(CAN_MODE_ID flag);
bool canctrl_CheckFlag(CAN_MODE_ID flag);
void canctrl_GetRxData(uint8_t *outData);
HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t RTR);
HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can, CAN_DEVICE_ID ID);
HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO);
HAL_StatusTypeDef canctrl_SetID(uint32_t ID);
HAL_StatusTypeDef canctrl_PutMessage(void* data,size_t dataSize);
HAL_StatusTypeDef canctrl_FilCfg(CAN_HandleTypeDef *can, uint32_t filterID, uint32_t filBank, uint32_t FIFO);
HAL_StatusTypeDef canctrl_Init(CAN_HandleTypeDef *can);
void convBigEndianToLittleEndian(uint8_t *data, size_t length);
#endif /* INC_CAN_CONTROL_H_ */
