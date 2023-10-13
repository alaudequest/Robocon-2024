/*
 * BoardParameter.h
 *
 *  Created on: Sep 23, 2023
 *      Author: KHOA
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_
#include "main.h"
#include "PID.h"
#include "AngleOptimizer.h"
#include "InverseKinematic.h"


typedef enum PID_type{
	PID_DC_SPEED = 1,
	PID_DC_ANGLE,
	PID_BLDC_SPEED,
}PID_type;



void brd_Init();
PID_Param brd_GetPID(PID_type type);
void brd_SetPID(PID_Param pid,PID_type type);

#endif /* INC_BOARDPARAMETER_H_ */
