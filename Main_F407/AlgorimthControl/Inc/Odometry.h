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

typedef struct odo_Param{
	slave_SpeadParam slave[3];
	zeta_Param zeta;
	CAN_SpeedBLDC_AngleDC slave_SpAg[3];
	float Vx[3];
	float Vy[3];
}odo_Param;

typedef enum Slave_ID{
	Slave1,
	Slave2,
	Slave3,
}Slave_ID;
void slave_SpeedRead(slave_SpeadParam *sp, float count);

slave_SpeadParam odo_getSpeedBLDC(Slave_ID ID);
void odo_SetObjSpeedBLDC(Slave_ID ID,slave_SpeadParam speed);

void odo_PosCal();

void odo_SetObj_SpAg(Slave_ID ID,CAN_SpeedBLDC_AngleDC spAg);

void odo_OmegaToZeta(zeta_Param *zeta, float *VxA, float *VyA);

#endif /* INC_ODOMETRY_H_ */
