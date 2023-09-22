/*
 * CAN_FuncHandle.h
 *
 *  Created on: Sep 22, 2023
 *      Author: KHOA
 */

#ifndef INC_CAN_FUNCHANDLE_H_
#define INC_CAN_FUNCHANDLE_H_
#include "main.h"

void canfunc_HandleRxEvent(void(*pCallback)(CAN_MODE_ID ID));
void canfunc_MotorBrake(bool brake);
HAL_StatusTypeDef canfunc_MotorPutEncoderPulse(int16_t encBLDC, int16_t encDC);
HAL_StatusTypeDef canfunc_MotorPutSpeedAndRotation(float speed, float angle);
void canfunc_MotorGetEncoderPulse(int16_t *encBLDC, int16_t *encDC);
void canfunc_MotorGetSpeedAndRotation(float *speed, float *angle);


#endif /* INC_CAN_FUNCHANDLE_H_ */
