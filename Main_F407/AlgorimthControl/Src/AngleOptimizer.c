/*
 * AngleOptimizer.c
 *
 *  Created on: Oct 12, 2023
 *      Author: Admin
 */

#include "AngleOptimizer.h"
float modulo360(float Angle){
	int Result = (int)Angle/360;
	return Angle-Result*360;
}

float modulo180(float Angle)
{
	int Result = (int)Angle/180;
	return Angle-Result*180;
}

void angopt_Cal(ModuleID ID,float input)
{
	Angle_Opt_Param opt = swer_GetOptAngle(ID);
#define direct 			opt.direct
#define currentAngle 	opt.currentAngle
#define deltaAngle 		opt.deltaAngle
#define outputAngle 	opt.outputAngle
#define preAngle 		opt.preAngle
#define calInput		opt.calInput
#define preCal			opt.preCal
#define deltaCal		opt.deltaCal
	if(input != preAngle)
	{
		calInput = input;
		if((currentAngle>=0)&&(calInput<0))calInput+=360;
		else if ((currentAngle<0)&&(calInput>0))calInput-=360;

		deltaCal = calInput-preCal;
		if(deltaCal>180)deltaCal+=-360;
		else if (deltaCal<-180)deltaCal+=360;

		if( ((deltaCal>90)&&(deltaCal<180))
		|| ((deltaCal<-90)&&(deltaCal>-180))
		|| ((deltaCal==180)) )	direct*=-1;

		deltaAngle = calInput - modulo360(currentAngle);

		if(deltaAngle>180)deltaAngle+=-360;
		else if(deltaAngle<-180)deltaAngle+=360;

		if((deltaAngle<=90)&&(deltaAngle>=-90))deltaAngle = deltaAngle;
		else if ((deltaAngle>90)&&(deltaAngle<=180))deltaAngle += -180.0;
		else if ((deltaAngle<-90)&&(deltaAngle>=-180))deltaAngle += 180.0;

		preAngle = input;
		preCal = calInput;
		currentAngle += deltaAngle;

		if(calInput == modulo360(currentAngle)) direct = 1;

		swer_SetOptAngle(ID, opt);
	}
}

