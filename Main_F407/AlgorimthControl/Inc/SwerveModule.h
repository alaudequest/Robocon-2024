/*
 * SwerveModule.h
 *
 *  Created on: Oct 13, 2023
 *      Author: Admin
 */

#ifndef INC_SWERVEMODULE_H_
#define INC_SWERVEMODULE_H_

#include "PID.h"
#include "math.h"

#define MAX_MODULE 					4
#define ROBOT_WHEEL_RADIUS_METER 	0.045

typedef enum ModuleID{
	MODULE_ID_1 = 1,
	MODULE_ID_2,
	MODULE_ID_3,
	MODULE_ID_4,
}ModuleID;

typedef struct WheelVector{
	float wheelVelX;
	float wheelVelY;
	float PreAngle;
}WheelVector;


typedef void (*pVectorCalXY)(float, float, float);
typedef struct SwerveModuleParam {
	WheelVector vct;
	pVectorCalXY pVctXY;
}SwerveModuleParam;

typedef enum PID_type{
	PID_DC_SPEED = 1,
	PID_DC_ANGLE,
	PID_BLDC_SPEED,
}PID_type;

WheelVector swer_GetWheelVector(ModuleID ID);
void swer_SetWheelVector(ModuleID ID,WheelVector vct);


pVectorCalXY swer_GetFuncHandle(ModuleID ID);
void swer_Init();


#endif /* INC_SWERVEMODULE_H_ */
