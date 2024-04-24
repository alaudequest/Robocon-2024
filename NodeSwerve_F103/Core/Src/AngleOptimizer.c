/*
 * AngleOptimizer.c
 *
 *  Created on: Apr 1, 2024
 *      Author: Admin
 */
#include"AngleOptimizer.h"
Angle_Opt_Param opt;


float absf(float num)
{
	if (num>=0)return num;
	else return num*-1;
}

float modulo360(float Angle){
	int Result = (int)Angle/360.0;
	return Angle-Result*360.0;
}

float modulo180(float Angle)
{
	int Result = (int)Angle/180.0;
	return Angle-Result*180.0;
}

int angopt_QuadrantCheckInput(float x, float y)
{
	float xT = x,yT = y;
	if(absf(xT)<0.0001) xT = 0;
	if(absf(yT)<0.0001) yT = 0;

	if((xT>0)&&(yT>0))return 1;
	else if((xT>0)&&(yT<0))return 2;
	else if((xT<0)&&(yT<0))return 3;
	else if((xT<0)&&(yT>0))return 4;
	else if((xT==0)&&(yT>0))return -1;
	else if((xT==0)&&(yT<0))return -2;
	else if((xT>0)&&(yT==0))return -3;
	else if((xT<0)&&(yT==0))return -4;

	return 0;
}

void angopt_QuadRantCheckOutput(float Input)
{

#define Case1		opt.Case1
#define Case2		opt.Case2
#define Direc 		opt.direct

	float XCurr = cos(Input);
	float YCurr = sin(Input);



	Case2 = angopt_QuadrantCheckInput(XCurr, YCurr);

	if (Case1 == 0)Direc = 0;

		else {
			if(Case2 == Case1)Direc = 1;
			else Direc = -1;
		}
}

int angopt_QuadRantCheckOutput2(float Input1,float Input2)
{
#define Case1		opt.Case1
#define Case2		opt.Case2
#define Direc 		opt.direct

	float XCurr1 = cos(Input1*M_PI/180);
	float YCurr1 = sin(Input1*M_PI/180);

	float XCurr2 = cos(Input2*M_PI/180);
	float YCurr2 = sin(Input2*M_PI/180);


	Case1 = angopt_QuadrantCheckInput(XCurr1, YCurr1);
	Case2 = angopt_QuadrantCheckInput(XCurr2, YCurr2);

	if (Case1 == 0)Direc = 0;

		else {
			if(Case2 == Case1)Direc = 1;
			else Direc = -1;
		}
	return Direc;
}

void angopt_Cal(float input)
{
#define direct 			opt.direct
#define currentAngle 	opt.currentAngle
#define deltaAngle 		opt.deltaAngle
#define outputAngle 	opt.outputAngle
#define preAngle 		opt.preAngle
#define calInput		opt.calInput
#define preCal			opt.preCal
#define deltaCal		opt.deltaCal
	if(input != preAngle){
		calInput = input;

		if((currentAngle>=0)&&(calInput<0))
		{
			calInput+=360;
		}
		else if ((currentAngle<0)&&(calInput>0))
		{
			calInput-=360;
		}

		deltaAngle = calInput - modulo360(currentAngle);

		if(deltaAngle>180)
		{
			deltaAngle+=-360;
		}
		else if(deltaAngle<-180)
		{
			deltaAngle+=360;
		}

		if((deltaAngle<=90)&&(deltaAngle>=-90))deltaAngle = deltaAngle;
		else if ((deltaAngle>90)&&(deltaAngle<=180))deltaAngle += -180.0;
		else if ((deltaAngle<-90)&&(deltaAngle>=-180))deltaAngle += 180.0;

		preAngle = input;
		preCal = calInput;
		currentAngle += deltaAngle;
		if(currentAngle>=1080) currentAngle-=360;
		else if (currentAngle<=-1080) currentAngle+=360;
	}
}

float angopt_GetOptAngle()
{
#define currentAngle 	opt.currentAngle
	return currentAngle;
}
