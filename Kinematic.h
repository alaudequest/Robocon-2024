/*
 * Kinematic.h
 *
 *  Created on: Sep 14, 2023
 *      Author: defaultuser0
 */

#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

#include "main.h"
#include "math.h"
#include "stdlib.h"


#define robot_Lenght 0.3
#define robot_WheelR 0.09

//Optimizer Swerve1;
typedef struct Optimizer{
	int Direc;
	float CurrentAngle;
	float DeltaAngle;
	float OutPutAngle;
	float PreAngle;
	float CalInput;
	float preCal;
	float DeltaCal;
	uint8_t OdoFlagX;
	uint8_t OdoFlagY;
	double Xcoef,Ycoef;
	double XCurrPos;
	double YCurrPos;
}Optimizer;

typedef struct Kinematic {
	float wheel_Vel_X;
	float wheel_Vel_Y;
	float wheel_Angle;
	float wheel_AngleVel;
	float u;
	float v;
	float rad;
	float c;
	int preQuad;
	int currQuad;
} Kinematic;

void SwerveInit(Optimizer *Swerve);
float modulo360(float Angle);
float modulo180(float Angle);
void OptimizeAngle (Optimizer *Swerve_Optimizer,float InPut);
int8_t quadrantCheck(float X, float Y);
void InverseKine(Kinematic *Swerve,Optimizer *Swerve_Optimizer,float u,float v ,float r);

#endif /* INC_KINEMATIC_H_ */
