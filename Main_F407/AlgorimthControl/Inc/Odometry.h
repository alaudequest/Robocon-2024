/*
 * Odometry.h
 *
 *  Created on: Feb 14, 2024
 *      Author: Admin
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "math.h"
#include "CAN_FuncHandle.h"
#define DELTA_T 0.05
#define PULSE_PER_REV 200*2.56

typedef struct slave_SpeadParam{
	float V;
	float Vx;
	float Vy;

	float VxC;
	float VyC;
	int preCount;
}slave_SpeadParam;

typedef struct zeta_Param{
	float uOut;
	float vOut;
	float rOut;
}zeta_Param;

typedef struct odo_Pose{
	float poseX;
	float poseY;
	float poseTheta;
	float poseTheta_Deg;
}odo_Pose;

typedef struct odo_Param{
	slave_SpeadParam slave[3];
	zeta_Param zeta;
	CAN_SpeedBLDC_AngleDC slave_SpAg[3];
	odo_Pose pose;
	float Vx[3];
	float Vy[3];
}odo_Param;

float odo_GetUout();
float odo_GetVout();
float odo_GetRout();

float odo_GetPoseX();
float odo_GetPoseY();
float odo_GetPoseTheta();

typedef enum Slave_ID{
	Slave1,
	Slave2,
	Slave3,
}Slave_ID;
void slave_SpeedRead(slave_SpeadParam *sp, float count);

slave_SpeadParam odo_GetSpeedBLDC(Slave_ID ID);
void odo_SetObjSpeedBLDC(Slave_ID ID,slave_SpeadParam speed);

odo_Pose odo_GetPose();
void odo_SetPose(odo_Pose pose);

zeta_Param odo_GetZeta();
void odo_SetZeta(zeta_Param zeta);

void odo_SetPoseTheta(float theta);
void odo_ResetPose();
void odo_ResetPose_2(bool *reset);

void odo_PosCal();

void odo_SetObj_SpAg(Slave_ID ID,CAN_SpeedBLDC_AngleDC spAg);

void odo_OmegaToZeta(zeta_Param *zeta, float *VxA, float *VyA);

void odo_EulerCal();

#endif /* INC_ODOMETRY_H_ */
