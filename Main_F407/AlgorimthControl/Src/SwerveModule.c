/*
 * SwerveModule.c
 *
 *  Created on: Oct 13, 2023
 *      Author: Admin
 */

#include "SwerveModule.h"

SwerveModuleParam swerParam[MAX_MODULE];
void swer1_CalVector(float u, float v, float r){	
	WheelVector vct;
	vct.wheelVelX = u - ROBOT_WIDTH_METER*r;
	vct.wheelVelY = v - ROBOT_LENGHT_METER*r;
	swer_SetWheelVector(MODULE_ID_1, vct);
}

void swer2_CalVector(float u, float v, float r){
	WheelVector vct;
	vct.wheelVelX = u + ROBOT_WIDTH_METER*r;
	vct.wheelVelY = v - ROBOT_LENGHT_METER*r;
	swer_SetWheelVector(MODULE_ID_2, vct);
}

void swer3_CalVector(float u, float v, float r){
	WheelVector vct;
	vct.wheelVelX = u - ROBOT_WIDTH_METER*r;
	vct.wheelVelY = v + ROBOT_LENGHT_METER*r;
	swer_SetWheelVector(MODULE_ID_3, vct);
	
}

void swer4_CalVector(float u, float v, float r){
	WheelVector vct;
	vct.wheelVelX = u + ROBOT_WIDTH_METER*r;
	vct.wheelVelY = v + ROBOT_LENGHT_METER*r;
	swer_SetWheelVector(MODULE_ID_4, vct);
}


void swer_Init(){
	swerParam[MODULE_ID_1].pVctXY = &swer1_CalVector;
	swerParam[MODULE_ID_2].pVctXY = &swer2_CalVector;
	swerParam[MODULE_ID_3].pVctXY = &swer3_CalVector;
	swerParam[MODULE_ID_4].pVctXY = &swer4_CalVector;
}


void swer_SetWheelVector(ModuleID ID,WheelVector vct){swerParam[ID].vct = vct;}
WheelVector swer_GetWheelVector(ModuleID ID){return swerParam[ID].vct;}
void swer_SetOptAngle(ModuleID ID,Angle_Opt_Param angleOpt){swerParam[ID].angleOpt = angleOpt;}
Angle_Opt_Param swer_GetOptAngle(ModuleID ID){return swerParam[ID].angleOpt;}
pVectorCalXY swer_GetFuncHandle(ModuleID ID){return *swerParam[ID].pVctXY;}

