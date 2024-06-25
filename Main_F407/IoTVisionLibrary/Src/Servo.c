/*
 * Servo.c
 *
 *  Created on: Mar 31, 2023
 *      Author: SpiritBoi
 */
#include "Servo.h"
#ifdef CONFIG_USE_SERVO

Servo *svTemp;

HAL_StatusTypeDef Servo_Config(Servo *sv, TIM_HandleTypeDef *htim, uint32_t TIM_CHANNEL)
{
	if(!sv) return HAL_ERROR;
	sv->htim = htim;
	sv->TIM_CHANNEL = TIM_CHANNEL;
	return HAL_OK;
}

HAL_StatusTypeDef Servo_Start()
{
	return SERVO_START;
}
HAL_StatusTypeDef Servo_SetTarget(Servo *sv)
{
	if(!sv) return HAL_ERROR;
	svTemp = sv;
	return HAL_OK;
}
HAL_StatusTypeDef Servo_SetAngle(uint8_t Angle)
{
	if(!SERVO_ANGLE_VALID(Angle)) return HAL_ERROR;
	SERVO_TURN(Angle);
	return HAL_OK;
}

#endif
