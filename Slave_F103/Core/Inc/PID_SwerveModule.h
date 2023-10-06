/*
 * PID_SwerveModule.h
 *
 *  Created on: Oct 3, 2023
 *      Author: KHOA
 */

#ifndef INC_PID_SWERVEMODULE_H_
#define INC_PID_SWERVEMODULE_H_

#include "main.h"
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "BoardParameter.h"


void PID_BLDC_CalSpeed(float Target_set);
void PID_DC_CalSpeed(float Target_set);
void PID_DC_CalPos(float Target_set);

#endif /* INC_PID_SWERVEMODULE_H_ */
