/*
 * SwerveModule.c
 *
 *  Created on: Oct 13, 2023
 *      Author: Admin
 */

#include "SwerveModule.h"
#define DX1	0.37545
#define DX2 -0.07171
#define DX3	-0.07171

#define DY1	0
#define DY2	0.23373/2
#define DY3	-0.23373/2

SwerveModuleParam swerParam[MAX_MODULE];
void swer0_CalVector(float u, float v, float r){
	__NOP();
}
//void swer1_CalVector(float u, float v, float r){
//	WheelVector vct;
//	float alpha = 315*M_PI/180;
//	vct.wheelVelX = cos(alpha)*(u + ROBOT_WIDTH_METER*r) - sin(alpha)*(v - ROBOT_LENGHT_METER*r);
//	vct.wheelVelY = sin(alpha)*(u + ROBOT_WIDTH_METER*r) + cos(alpha)*(v - ROBOT_LENGHT_METER*r);
//	swer_SetWheelVector(MODULE_ID_1, vct);
//}
//
//void swer2_CalVector(float u, float v, float r){
//	WheelVector vct;
//	float alpha = 45*M_PI/180;
//	vct.wheelVelX = cos(alpha)*(u - ROBOT_WIDTH_METER*r) - sin(alpha)*(v - ROBOT_LENGHT_METER*r);
//	vct.wheelVelY = sin(alpha)*(u - ROBOT_WIDTH_METER*r) + cos(alpha)*(v - ROBOT_LENGHT_METER*r);
//	swer_SetWheelVector(MODULE_ID_2, vct);
//}
//
//void swer3_CalVector(float u, float v, float r){
//	WheelVector vct;
//	float alpha = 135*M_PI/180;
//	vct.wheelVelX = cos(alpha)*(u - ROBOT_WIDTH_METER*r) - sin(alpha)*(v + ROBOT_LENGHT_METER*r);
//	vct.wheelVelY = sin(alpha)*(u - ROBOT_WIDTH_METER*r) + cos(alpha)*(v + ROBOT_LENGHT_METER*r);
//	swer_SetWheelVector(MODULE_ID_3, vct);
//
//}
//
//void swer4_CalVector(float u, float v, float r){
//	WheelVector vct;
//	float alpha = (225)*M_PI/180;
//	vct.wheelVelX = cos(alpha)*(u + ROBOT_WIDTH_METER*r) - sin(alpha)*(v + ROBOT_LENGHT_METER*r);
//	vct.wheelVelY = sin(alpha)*(u + ROBOT_WIDTH_METER*r) + cos(alpha)*(v + ROBOT_LENGHT_METER*r);
//	swer_SetWheelVector(MODULE_ID_4, vct);
//}
void swer1_CalVector(float u, float v, float r){
	WheelVector vct;
	float alpha = 315*M_PI/180;
	vct.wheelVelX = cos(alpha)*(u + DX1*r) - sin(alpha)*(v - DY1*r);
	vct.wheelVelY = sin(alpha)*(u + DX1*r) + cos(alpha)*(v - DY1*r);
	swer_SetWheelVector(MODULE_ID_1, vct);
}

void swer2_CalVector(float u, float v, float r){
	WheelVector vct;
	float alpha = 45*M_PI/180;
	vct.wheelVelX = cos(alpha)*(u + DX2*r) - sin(alpha)*(v - DY2*r);
	vct.wheelVelY = sin(alpha)*(u + DX2*r) + cos(alpha)*(v - DY2*r);
	swer_SetWheelVector(MODULE_ID_2, vct);
}

void swer3_CalVector(float u, float v, float r){
	WheelVector vct;
	float alpha = 135*M_PI/180;
	vct.wheelVelX = cos(alpha)*(u + DX3*r) - sin(alpha)*(v - DY3*r);
	vct.wheelVelY = sin(alpha)*(u + DX3*r) + cos(alpha)*(v - DY3*r);
	swer_SetWheelVector(MODULE_ID_3, vct);
	
}

void swer_Init(){
	swerParam[0].pVctXY = &swer0_CalVector;
	swerParam[MODULE_ID_1].pVctXY = &swer1_CalVector;
	swerParam[MODULE_ID_2].pVctXY = &swer2_CalVector;
	swerParam[MODULE_ID_3].pVctXY = &swer3_CalVector;
//	swerParam[MODULE_ID_4].pVctXY = &swer4_CalVector;

	swerParam[MODULE_ID_1].angleOpt.direct =
	swerParam[MODULE_ID_2].angleOpt.direct =
	swerParam[MODULE_ID_3].angleOpt.direct = 1;

//	swerParam[MODULE_ID_1].angleOpt.OffsetAngle = 315;
//	swerParam[MODULE_ID_2].angleOpt.OffsetAngle = 45;
//	swerParam[MODULE_ID_3].angleOpt.OffsetAngle = 135;
//	swerParam[MODULE_ID_4].angleOpt.OffsetAngle = 225;

}


void swer_SetWheelVector(ModuleID ID,WheelVector vct){swerParam[ID].vct = vct;}
WheelVector swer_GetWheelVector(ModuleID ID){return swerParam[ID].vct;}
void swer_SetOptAngle(ModuleID ID,Angle_Opt_Param angleOpt){swerParam[ID].angleOpt = angleOpt;}
Angle_Opt_Param swer_GetOptAngle(ModuleID ID){return swerParam[ID].angleOpt;}
pVectorCalXY swer_GetFuncHandle(ModuleID ID){return *swerParam[ID].pVctXY;}

