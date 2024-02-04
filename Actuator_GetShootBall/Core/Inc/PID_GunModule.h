/*
 * PID_GunModule.h
 *
 *  Created on: Jan 21, 2024
 *      Author: namdhay
 */

#ifndef INC_PID_GUNMODULE_H_
#define INC_PID_GUNMODULE_H_

#include "main.h"
#include "BoardParameter.h"
#include "stdbool.h"
#include "cmsis_os.h"

void PID_Rotary_CalSpeed(float Target_set);
void PID_Rotary_CalPos(float Target_set);
void PID_Gun_CalSpeed(float Target_set, Motor_Type gun);
void PID_RuloBall_CalSpeed(float Target_set, Motor_Type rulo);
void PID_Motor_Stop_All();
#endif /* INC_PID_GUNMODULE_H_ */
