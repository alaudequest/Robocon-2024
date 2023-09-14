/*
 * CAN_Control.h
 *
 *  Created on: Sep 12, 2023
 *      Author: KHOA
 */

#ifndef INC_CAN_CONTROL_H_
#define INC_CAN_CONTROL_H_
#include "main.h"

typedef enum listID {
	CAN_ID_BRAKE,
	CAN_ID_SLAVE,
	CAN_ID_RUN_ROTATE_MOTOR,
	CAN_ID_ENCODER,
}listID;

HAL_StatusTypeDef canctrl_MakeStdTxHeader(uint16_t ID, uint32_t DLC, uint32_t RTR);
HAL_StatusTypeDef canctrl_Send(CAN_HandleTypeDef *can,uint8_t *txdata, uint16_t txDataSize);
HAL_StatusTypeDef canctrl_Receive(CAN_HandleTypeDef *can,uint8_t *rxdata, uint32_t FIFO);
HAL_StatusTypeDef canctrl_SetFilter(CAN_HandleTypeDef *can, uint32_t FIFO, uint32_t ID);
#endif /* INC_CAN_CONTROL_H_ */
