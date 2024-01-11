/*
 * SwerveOdometry.c
 *
 *  Created on: Dec 24, 2023
 *      Author: Admin
 */

#include "SwerveOdometry.h"
#include "CAN_FuncHandle.h"
//#include "main.h"
//#include "BoardParameter.h"

#define PULSEPERREVOL 400*2.5
#define Ts 0.05
#define WHEELRADIUS 0.0045
SwerveOdoParam OdoParam;


void SpeedRead(ReadSpeedSlave *sp, float count)
{
	sp->V = ((((count-sp->preCount)/Ts)/PULSEPERREVOL)*2*M_PI)*WHEELRADIUS;
	sp->Vfilt = (1-sp->filterAlpha)*sp->VfiltPre+sp->filterAlpha*sp->V;
	sp->VfiltPre = sp->Vfilt;

	sp->preCount = count;
}

void Forwardkinecal(ForwardKine *kine, float* VxA, float* VyA)
{
	kine->uOut = 0.3333*VxA[0] - 0*VyA[0] + 0.3333*VxA[1] + 0*VyA[1] - 0.3333*VxA[2] + 0*VyA[2] ;
	kine->vOut = 0*VxA[0] + 0.1898*VyA[0] - 0.0563*VxA[1] + 0.4051*VyA[1] - -0.0563*VxA[2] - 0.4051*VyA[2] ;
	kine->thetaOut = 0*VxA[0] - 1.8560*VyA[0] + -0.7276*VxA[1] - -0.9280*VyA[1] + 0.7276*VxA[2] - -0.9280*VyA[2] ;
}

void OdometryInit(){
	OdoParam.Odo.Suy = 2*M_PI*0.045/PULSEPERREVOL;

}
void Odometry(CAN_SpeedBLDC_AngleDC *SpeedAngle){
	for (int i = 0; i < 3; i ++)
	{
		SpeedRead(&OdoParam.Module[i] , SpeedAngle[i].bldcSpeed);
		OdoParam.Module[i].Vx = OdoParam.Module[i].V*cos(SpeedAngle[i].dcAngle*M_PI);
		OdoParam.Module[i].Vy = OdoParam.Module[i].V*sin(SpeedAngle[i].dcAngle*M_PI);
		OdoParam.Vx[i] = OdoParam.Module[i].Vx;
		OdoParam.Vy[i] = OdoParam.Module[i].Vy;
	}
	Forwardkinecal(&OdoParam.kine, OdoParam.Vx, OdoParam.Vy);
	OdoParam.Odo.dX = OdoParam.kine.uOut*Ts;
	OdoParam.Odo.dY = OdoParam.kine.vOut*Ts;
	OdoParam.Odo.dTheta = OdoParam.kine.thetaOut*Ts;
//
	float sinTheta = sin(OdoParam.Odo.dTheta);
	float cosTheta = cos(OdoParam.Odo.dTheta);
//
	if(fabs(OdoParam.Odo.dTheta)<0.00001){
		OdoParam.Odo.S = 1 -((pow(OdoParam.Odo.dTheta,2))/6);
		OdoParam.Odo.C = -0.5*OdoParam.Odo.dTheta;
	}else{
		OdoParam.Odo.S = sinTheta/OdoParam.Odo.dTheta;
		OdoParam.Odo.C = (1-cosTheta)/OdoParam.Odo.dTheta;
	}
	OdoParam.Odo.poseX += (cos(OdoParam.Odo.OffsetGyro)*(OdoParam.Odo.dX*OdoParam.Odo.S-OdoParam.Odo.dY*OdoParam.Odo.C)-sin(OdoParam.Odo.OffsetGyro)*(OdoParam.Odo.dX*OdoParam.Odo.C+OdoParam.Odo.dY*OdoParam.Odo.S));
	OdoParam.Odo.poseY += (sin(OdoParam.Odo.OffsetGyro)*(OdoParam.Odo.dX*OdoParam.Odo.S-OdoParam.Odo.dY*OdoParam.Odo.C)+cos(OdoParam.Odo.OffsetGyro)*(OdoParam.Odo.dX*OdoParam.Odo.C+OdoParam.Odo.dY*OdoParam.Odo.S));
	OdoParam.Odo.poseTheta += OdoParam.Odo.dTheta;
}
//void GetbldcSpeed(int speed,int i){
//	OdoParam.SpeedAngle[i].bldcSpeed = speed;
//}
//void GetDcAngle(float dcAngle,int i){
//	OdoParam.SpeedAngle[i].dcAngle = dcAngle;
//}
//ReadSpeedSlave Module[4];
