/*
 * ForwardKinematic.h
 *
 *  Created on: Nov 11, 2023
 *      Author: Admin
 */

#ifndef INC_FORWARDKINEMATIC_H_
#define INC_FORWARDKINEMATIC_H_

#include "SwerveModule.h"

typedef struct SwerveForwardVeloc{
	float vx;
	float vy;
}SwerveForwardVeloc;

typedef struct SwerveForwardOutput{
	float uOut;
	float vOut;
	float rOut;
}SwerveForwardOutput;
typedef struct SwerveForwardKinematic{
	SwerveForwardVeloc v1;
	SwerveForwardVeloc v2;
	SwerveForwardVeloc v3;
	SwerveForwardVeloc v4;

	SwerveForwardOutput Output;
}SwerveForwardKinematic;

SwerveForwardVeloc getVelocWheel(ModuleID ID);
void setVelocWheel(ModuleID ID,float Vx,float Vy);

SwerveForwardOutput getOutputValue();
void setOutputValue(float u ,float v ,float r);

void ForwardKinematicCal();

#endif /* INC_FORWARDKINEMATIC_H_ */
