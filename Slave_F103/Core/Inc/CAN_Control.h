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

typedef enum Command_ID_List{
	CANCTRL_ID_SPEED_ANGLE_MOTOR_POS = 1,
	CANCTRL_ID_ENCODER_POS = 2,
	CANCTRL_ID_DEVICE_POS = 3,
	CANCTRL_ID_BRAKE_MOTOR_POS = 6,
	CANCTRL_ID_SET_HOME = 7,
}Command_ID_List;

#define CAN_ID_ENCODER_MASK (1 << CANCTRL_ID_ENCODER_POS)
#define CAN_ID_BRAKE_MASK (1 << CANCTRL_ID_BRAKE_MOTOR_POS)
#define CAN_ID_SPEED_ANGLE_MASK (1 << CANCTRL_ID_SPEED_ANGLE_MOTOR_POS)
#define CAN_ID_SET_HOME (1 << CANCTRL_ID_SET_HOME)
typedef enum CAN_ID{
	CANCTRL_ID_MOTOR_CONTROLLER_1 = 1,
	CANCTRL_ID_MOTOR_CONTROLLER_2,
	CANCTRL_ID_MOTOR_CONTROLLER_3,
	CANCTRL_ID_MOTOR_CONTROLLER_4,
	CANCTRL_ID_MAIN,
}CAN_ID;

typedef enum CAN_EVT{
	CAN_EVT_RX_FIFO0,
	CAN_EVT_RX_FIFO1,
	CAN_EVT_SET_HOME,
	CAN_EVT_SPEED_ANGLE,
	CAN_EVT_BRAKE_MOTOR,
	CAN_EVT_GET_ENCODER,
}CAN_EVT;
CAN_RxHeaderTypeDef canctrl_GetRxHeader();
float canctrl_GetFloatNum();
uint64_t canctrl_GetIntNum();
uint32_t canctrl_GetDLC();
uint32_t canctrl_GetID();
void canctrl_SetDLC(uint8_t DLC);
void canctrl_RTR_SetToData();
void canctrl_RTR_SetToRemote();
void canctrl_SetFlag(CAN_EVT flag);
void canctrl_ClearFlag(CAN_EVT flag);
bool canctrl_CheckFlag(CAN_EVT flag);
void canctrl_MotorSendBrakeMessage(CAN_HandleTypeDef *can, CAN_ID motorCtrlID, bool brake);
void canctrl_MotorGetEncoderPulse(int16_t *encBLDC, int16_t *encDC);
void canctrl_MotorGetSpeedAndRotation(float *speed, float *angle);
HAL_StatusTypeDef canctrl_MotorPutEncoderPulse(uint8_t slaveID, int16_t encBLDC, int16_t encDC);
HAL_StatusTypeDef canctrl_MotorSetSpeedAndRotation(uint8_t slaveID, float speed, float angle);
HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t RTR);
HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can, uint32_t ID);
HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO);
HAL_StatusTypeDef canctrl_SetID(uint32_t ID);
HAL_StatusTypeDef canctrl_PutMessage(void* data,size_t dataSize);
HAL_StatusTypeDef canctrl_FilCfg(CAN_HandleTypeDef *can, uint32_t filterID, uint32_t filBank, uint32_t FIFO);
HAL_StatusTypeDef canctrl_Init(CAN_HandleTypeDef *can);
#endif /* INC_CAN_CONTROL_H_ */
