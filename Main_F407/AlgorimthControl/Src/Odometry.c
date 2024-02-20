/*
 * Odometry.c
 *
 *  Created on: Feb 14, 2024
 *      Author: Admin
 */


#include "Odometry.h"

odo_Param odo;

void slave_SpeedRead(slave_SpeadParam *sp, float count){
	sp->V = 0.045*2*M_PI*((count-sp->preCount)/DELTA_T)/(PULSE_PER_REV*4);
	sp->preCount = count;
}

slave_SpeadParam odo_getSpeedBLDC_1(Slave_ID ID){return odo.slave[ID];}
void odo_SetObjSpeedBLDC(Slave_ID ID,slave_SpeadParam speed){odo.slave[ID] = speed;}

void odo_SetObj_SpAg(Slave_ID ID,CAN_SpeedBLDC_AngleDC spAg){odo.slave_SpAg[ID] = spAg;}

void odo_OmegaToZeta(zeta_Param *zeta, float* VxA, float* VyA)
{
	zeta->uOut 		= - 0.2357*VxA[0] + 0.2357*VyA[0] - 0.2357*VxA[1] - 0.2357*VyA[1] + 0.3333*VxA[2] + 0.0000*VyA[2] ;
	zeta->vOut 		= - 0.2467*VxA[0] - 0.3262*VyA[0] + 0.2467*VxA[1] - 0.3262*VyA[1] - 0.0000*VxA[2] + 0.1898*VyA[2] ;
	zeta->rOut 	    =   0.1417*VxA[0] + 1.1707*VyA[0] - 0.1417*VxA[1] + 1.1707*VyA[1] + 0.0000*VxA[2] + 1.8560*VyA[2] ;
}

float odo_GetUout(){return odo.zeta.uOut;}
float odo_GetVout(){return odo.zeta.vOut;}
float odo_GetRout(){return odo.zeta.rOut;}

float odo_GetPoseX(){return odo.pose.poseX;}
float odo_GetPoseY(){return odo.pose.poseY;}
float odo_GetPoseTheta(){return odo.pose.poseTheta;}

void odo_EulerCal()
{
	odo.pose.poseTheta += odo.zeta.rOut*DELTA_T;
	odo.pose.poseX 	   += (odo.zeta.uOut*cos(odo.pose.poseTheta) - odo.zeta.vOut*sin(odo.pose.poseTheta))*DELTA_T;
	odo.pose.poseY 	   += (odo.zeta.uOut*sin(odo.pose.poseTheta) + odo.zeta.vOut*cos(odo.pose.poseTheta))*DELTA_T;
}

void odo_PosCal(){
	for (int i = 0;i<=Slave3;i++){
		slave_SpeedRead(&odo.slave[i], odo.slave_SpAg[i].bldcSpeed);
		odo.slave[i].Vx = odo.slave[i].V * cos( odo.slave_SpAg[i].dcAngle*M_PI/180);
		odo.slave[i].Vy = odo.slave[i].V * sin( odo.slave_SpAg[i].dcAngle*M_PI/180);
		odo.Vx[i] = odo.slave[i].Vx;
		odo.Vy[i] = odo.slave[i].Vy;
	}

	odo_OmegaToZeta(&odo.zeta, odo.Vx, odo.Vy);

	odo_EulerCal();
}

odo_Pose odo_GetPose(){
	return odo.pose;
}
void odo_SetPose(odo_Pose pose){
	odo.pose = pose;
}

zeta_Param odo_GetZeta(){return odo.zeta;}
void odo_SetZeta(zeta_Param zeta){odo.zeta = zeta;}

void odo_SetPoseTheta(float theta){
	odo.pose.poseTheta = theta;
}
void odo_ResetPose(){
	odo.pose.poseTheta = 0;
	odo.pose.poseX = 0;
	odo.pose.poseY = 0;
}
