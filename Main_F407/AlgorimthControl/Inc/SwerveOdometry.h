/*
 * SwerveOdometry.h
 *
 *  Created on: Dec 24, 2023
 *      Author: Admin
 */

#ifndef INC_SWERVEODOMETRY_H_
#define INC_SWERVEODOMETRY_H_
//#include "main.h"
#include "math.h"
//#include "CAN_FuncHandle.h"
typedef struct ReadSpeedSlave{
	float V;
	float Vx;
	float Vy;

	float VxC;
	float VyC;

	float Offset;

	float Vfilt;
	float VfiltPre;
	float filterAlpha;

	int preCount;
}ReadSpeedSlave;

typedef struct ForwardKine{
	float uOut;
	float vOut;
	float thetaOut;
}ForwardKine;

typedef struct SwerveOdoHandle{
	float dX;
	float dY;
	float dTheta;

	float poseX;
	float poseY;
	float poseTheta;

	float S;
	float C;

	float OffsetGyro;
	float Suy;
}SwerveOdoHandle;

typedef struct SlaveSpeedAngle{
	int bldcSpeed;
	float dcAngle;
}SlaveSpeedAngle;


typedef struct SwerveOdoParam{
	float Vx[3];
	float Vy[3];
	ReadSpeedSlave Module[3];
	SwerveOdoHandle Odo;
	ForwardKine kine;

}SwerveOdoParam;

void ReadSpeed();
void ForwardCal();
void Odometry();
//void GetbldcSpeed(int speed,int i);
//void GetDcAngle(float dcAngle,int i);

#endif /* INC_SWERVEODOMETRY_H_ */
