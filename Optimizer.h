/*
 * Optimizer.h
 *
 *  Created on: Sep 14, 2023
 *      Author: defaultuser0
 */

#ifndef INC_OPTIMIZER_H_
#define INC_OPTIMIZER_H_

#include "main.h"

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

void SwerveInit(Optimizer *Swerve);
float modulo360(float Angle);
float modulo180(float Angle);
void OptimizeAngle (Optimizer *Swerve,float InPut);


#endif /* INC_OPTIMIZER_H_ */
