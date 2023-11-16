/*
 * ForwardKinematic.c
 *
 *  Created on: Nov 11, 2023
 *      Author: Admin
 */
#include "ForwardKinematic.h"

SwerveForwardKinematic Fkine;
#define MATRIX_COEF1	0.1768
#define MATRIX_COEF2 	0
#define MATRIX_COEF3 	0.7071

#define	V1				Fkine.v1
#define	V2				Fkine.v2
#define	V3				Fkine.v3
#define	V4				Fkine.v4

#define Vx1				Fkine.v1.vx
#define Vx2				Fkine.v2.vx
#define Vx3				Fkine.v3.vx
#define Vx4				Fkine.v4.vx

#define Vy1				Fkine.v1.vy
#define Vy2				Fkine.v2.vy
#define Vy3				Fkine.v3.vy
#define Vy4				Fkine.v4.vy

#define U				Fkine.Output.uOut
#define V				Fkine.Output.vOut
#define R				Fkine.Output.rOut

SwerveForwardVeloc getVelocWheel(ModuleID ID){
	switch (ID) {
		case MODULE_ID_1:
			return Fkine.v1;
			break;
		case MODULE_ID_2:
			return Fkine.v2;
			break;
		case MODULE_ID_3:
			return Fkine.v3;
			break;
		case MODULE_ID_4:
			return Fkine.v4;
			break;
		default:
			break;
	}
	return Fkine.v1;
}

void setVelocWheel(ModuleID ID,float Vx,float Vy)
{
	switch (ID) {
		case MODULE_ID_1:
			Fkine.v1.vx = Vx;
			Fkine.v1.vy = Vy;
			break;
		case MODULE_ID_2:
			Fkine.v2.vx = Vx;
			Fkine.v2.vy = Vy;
			break;
		case MODULE_ID_3:
			Fkine.v3.vx = Vx;
			Fkine.v3.vy = Vy;
			break;
		case MODULE_ID_4:
			Fkine.v4.vx = Vx;
			Fkine.v4.vy = Vy;
			break;
		default:
			break;
	}
}

SwerveForwardOutput getOutputValue(){
	return Fkine.Output;
}

void setOutputValue(float u ,float v ,float r){
	U = u;
	V = v;
	R = r;
}

void ForwardKinematicCal(){
	U = MATRIX_COEF1*Vx1 - MATRIX_COEF1*Vy1 + MATRIX_COEF1*Vx2 + MATRIX_COEF1*Vy2 - MATRIX_COEF1*Vx3 + MATRIX_COEF1*Vy3 - MATRIX_COEF1*Vx4 - MATRIX_COEF1*Vy4;
	V = MATRIX_COEF1*Vx1 + MATRIX_COEF1*Vy1 - MATRIX_COEF1*Vx2 + MATRIX_COEF1*Vy2 - MATRIX_COEF1*Vx3 - MATRIX_COEF1*Vy3 + MATRIX_COEF1*Vx4 - MATRIX_COEF1*Vy4;
	R = MATRIX_COEF2*Vx1 - MATRIX_COEF3*Vy1 + MATRIX_COEF2*Vx2 + MATRIX_COEF2*Vy2 - MATRIX_COEF2*Vx3 - MATRIX_COEF3*Vy3 - MATRIX_COEF2*Vx4 - MATRIX_COEF3*Vy4;
}
