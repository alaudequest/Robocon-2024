/*
 * SwerveModule.c
 *
 *  Created on: Oct 13, 2023
 *      Author: Admin
 */

#include "SwerveModule.h"
#define DX3	0.37545
#define DX1 -0.07171
#define DX2	-0.07171

#define DY3	0.0000
#define DY1	-0.23373/2
#define DY2	0.23373/2

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
	float alpha = 135*M_PI/180;
	vct.wheelVelX = cos(alpha)*(u - DY1*r) - sin(alpha)*(v + DX1*r);
	vct.wheelVelY = sin(alpha)*(u - DY1*r) + cos(alpha)*(v + DX1*r);
	swer_SetWheelVector(MODULE_ID_1, vct);
}

void swer2_CalVector(float u, float v, float r){
	WheelVector vct;
	float alpha = 225*M_PI/180;
	vct.wheelVelX = cos(alpha)*(u - DY2*r) - sin(alpha)*(v + DX2*r);
	vct.wheelVelY = sin(alpha)*(u - DY2*r) + cos(alpha)*(v + DX2*r);
	swer_SetWheelVector(MODULE_ID_2, vct);
}

void swer3_CalVector(float u, float v, float r){
	WheelVector vct;
	float alpha = 0*M_PI/180;
	vct.wheelVelX = cos(alpha)*(u - DY3*r) - sin(alpha)*(v + DX3*r);
	vct.wheelVelY = sin(alpha)*(u - DY3*r) + cos(alpha)*(v + DX3*r);
	swer_SetWheelVector(MODULE_ID_3, vct);
}

void swer_Init(){
	swerParam[0].pVctXY = &swer0_CalVector;
	swerParam[MODULE_ID_1].pVctXY = &swer1_CalVector;
	swerParam[MODULE_ID_2].pVctXY = &swer2_CalVector;
	swerParam[MODULE_ID_3].pVctXY = &swer3_CalVector;
}


void swer_SetWheelVector(ModuleID ID,WheelVector vct){swerParam[ID].vct = vct;}
WheelVector swer_GetWheelVector(ModuleID ID){return swerParam[ID].vct;}
pVectorCalXY swer_GetFuncHandle(ModuleID ID){return *swerParam[ID].pVctXY;}

