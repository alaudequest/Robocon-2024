/*
 * PIDPosition.h
 *
 *  Created on: Nov 1, 2023
 *      Author: Hoang May
 */

#ifndef INC_PIDPOSITION_H_
#define INC_PIDPOSITION_H_

#include "PID.h"
#include "OdometerHandle.h"
#include "LowPass.h"

#define	posDeltaT 0.05

typedef struct PID_POS {
	PID_Param pidPos_x;
	PID_Param pidPos_y;
	PID_Param pidPos_theta;

	lowPassParam pidPos_x_fil;
	lowPassParam pidPos_y_fil;
	lowPassParam pidPos_theta_fil;
}PID_POS;


typedef enum PID_POS_type{
	PID_POS_X = 1,
	PID_POS_Y,
	PID_POS_THETA,
}PID_POS_type;

void PID_Pos_Init(void);
float PID_CalPos_x(float Target_set);
float PID_CalPos_y(float Target_set);
float PID_CalPos_theta(float Target_set);
void PID_CalPos_SetPID(PID_Param pid,PID_POS_type type);
PID_Param PID_CalPos_GetPID(PID_POS_type type);

void PID_CalPos_Setfil(lowPassParam fil,PID_POS_type type);
lowPassParam PID_CalPos_Getfil(PID_POS_type type);

#endif /* INC_PIDPOSITION_H_ */
