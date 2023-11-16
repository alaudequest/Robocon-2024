/*
 * SwerveDriveOdometry.c
 *
 *  Created on: Nov 11, 2023
 *      Author: Admin
 */

#include "SwerveDriveOdometry.h"

SwerveDriveOdometry Sodo;

#define dX			Sodo.deltaX
#define dY			Sodo.deltaY
#define dTheta		Sodo.deltaTheta

#define currX		Sodo.currX
#define currY		Sodo.currY
#define currTheta	Sodo.currTheta

#define preX		Sodo.preX
#define preY		Sodo.preY
#define preTheta	Sodo.preTheta

#define s			Sodo.s
#define c			Sodo.c

#define poseX		Sodo.poseX
#define poseY		Sodo.poseY
#define poseTheta	Sodo.poseTheta

#define OffsetGyro	Sodo.OffsetGyro

void SwerveOdo_init(){

}
void SwerveOdo_PoseCal(float u, float v, float theta)
{
	dX = currX - preX;
	dY = currY - preY;
	dTheta = currTheta - preTheta;

	preX = currX;
	preY = currY;
	preTheta = currTheta;

	float sinTheta = sin(dTheta);
	float cosTheta = cos(dTheta);

	if(fabsf(dTheta)<0.0001){
		S = 1-((pow(dTheta,2))/6);
		C = 0.5*dTheta;
	}else{
		S = sinTheta/dTheta;
		C = (1-cosTheta)/dTheta;
	}

	poseX += cos(OffsetGyro)*(dX*S-dY*C)-sin(OffsetGyro)*(dX*C+dY*S);
	poseY += sin(OffsetGyro)*(dX*C+dY*S)+cos(OffsetGyro)*(dX*S-dY*C);
	poseTheta = theta;

}
