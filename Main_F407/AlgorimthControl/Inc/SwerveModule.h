/*
 * SwerveModule.h
 *
 *  Created on: Oct 13, 2023
 *      Author: Admin
 */

#ifndef INC_SWERVEMODULE_H_
#define INC_SWERVEMODULE_H_

#define MAX_MODULE 					4
#define ROBOT_WIDTH_METER 			0.25
#define ROBOT_LENGHT_METER			0.25
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
}WheelVector;

typedef struct Angle_Opt_Param{
	int direct;
	float currentAngle;
	float deltaAngle;
	float outputAngle;
	float preAngle;
	float calInput;
	float preCal;
	float deltaCal;
}Angle_Opt_Param;

typedef void (*pVectorCalXY)(float, float, float);
typedef struct SwerveModuleParam {
	Angle_Opt_Param angleOpt;
	WheelVector vct;
	pVectorCalXY pVctXY;
}SwerveModuleParam;

WheelVector swer_GetWheelVector(ModuleID ID);
void swer_SetWheelVector(ModuleID ID,WheelVector vct);

Angle_Opt_Param swer_GetOptAngle(ModuleID ID);
void swer_SetOptAngle(ModuleID ID,Angle_Opt_Param angleOpt);
pVectorCalXY swer_GetFuncHandle(ModuleID ID);
void swer_Init();
#endif /* INC_SWERVEMODULE_H_ */
