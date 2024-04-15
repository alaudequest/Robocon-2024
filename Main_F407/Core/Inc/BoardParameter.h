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
#include "InverseKinematic.h"

typedef enum MainF4Robot1TypePID {
	PID_RULO_1 = 1,
	PID_RULO_2,
	PID_ROBOT_THETA,
} MainF4Robot1TypePID;

#endif /* INC_BOARDPARAMETER_H_ */
