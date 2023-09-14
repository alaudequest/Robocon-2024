/*
 * CAN_Control.h
 *
 *  Created on: Sep 12, 2023
 *      Author: KHOA
 */

#ifndef INC_CAN_CONTROL_H_
#define INC_CAN_CONTROL_H_
#include "main.h"


HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t DLC, uint32_t RTR);
HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can);
HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can, uint32_t FIFO);
void canctrl_SetDLC(uint8_t DLC);
uint32_t canctrl_GetDLC();
uint32_t canctrl_GetID();
HAL_StatusTypeDef canctrl_SetID(uint32_t ID);
HAL_StatusTypeDef canctrl_PutMessage(uint64_t data);
void canctrl_RTR_SetToData();
void canctrl_RTR_SetToRemote();
CAN_RxHeaderTypeDef canctrl_GetRxHeader();
HAL_StatusTypeDef canctrl_FilCfg(CAN_HandleTypeDef *can, uint32_t filterID, uint32_t filBank, uint32_t FIFO);
HAL_StatusTypeDef canctrl_Init(CAN_HandleTypeDef *can);
#endif /* INC_CAN_CONTROL_H_ */
