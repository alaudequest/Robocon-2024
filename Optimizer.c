/*
 * Optimizer.c
 *
 *  Created on: Sep 14, 2023
 *      Author: defaultuser0
 */
#include "Optimizer.h"

//Khoi tao chi so cho moi cum Swerve
void SwerveInit(Optimizer *Swerve){
	Swerve -> Direc = 1;
	Swerve -> CurrentAngle = 0;
	Swerve -> DeltaAngle = 0;
	Swerve -> OutPutAngle = 0;
	Swerve -> PreAngle =0;
	Swerve -> preCal =0;
	Swerve -> DeltaCal = 0;
	Swerve->  CalInput = 0;
	Swerve-> OdoFlagX = 1;
	Swerve-> OdoFlagY = 1;
	Swerve-> Xcoef = 0;
	Swerve-> Ycoef = 0;
}



//Ham tinh toan chia het cho 360
float modulo360(float Angle)
{
	int Result = (int)Angle/360;
	return Angle-Result*360;
}



//Ham tinh toan chia het cho 180
float modulo180(float Angle)
{
	int Result = (int)Angle/180;
	return Angle-Result*180;
}



//Ham Tim Goc Xoay Toi Uu nhat cho cum Swerve
void OptimizeAngle (Optimizer *Swerve,float InPut)
{
	if (InPut != Swerve->PreAngle){

		Swerve->CalInput = InPut;
		if ((Swerve->CurrentAngle>=0)&&Swerve->CalInput<0)Swerve->CalInput+=360;
		else if ((Swerve->CurrentAngle<0)&&Swerve->CalInput>0) Swerve->CalInput-=360;

		Swerve->DeltaCal = Swerve->CalInput-Swerve->preCal;
		if(Swerve->DeltaCal>180)Swerve->DeltaCal+=-360;
		else if(Swerve->DeltaCal<-180)Swerve->DeltaCal+=360;


		if ((Swerve->DeltaCal>90)&&(Swerve->DeltaCal<180))
		{
			Swerve->Direc *= -1;
		}else if ((Swerve->DeltaCal<-90)&&(Swerve->DeltaCal>-180))
		{
			Swerve->Direc *= -1;
		}else if(abs(Swerve->DeltaCal)==180)
		{
			Swerve->Direc *= -1;
		}

		Swerve->DeltaAngle = Swerve->CalInput-modulo360(Swerve->CurrentAngle);

		if(Swerve->DeltaAngle>180)Swerve->DeltaAngle+=-360;
		else if(Swerve->DeltaAngle<-180)Swerve->DeltaAngle+=360;

		if((Swerve->DeltaAngle<=90)&&(Swerve->DeltaAngle>=-90))
		{
			Swerve->DeltaAngle = Swerve->DeltaAngle;
		}else if ((Swerve->DeltaAngle>90)&&(Swerve->DeltaAngle<=180))
		{
			Swerve->DeltaAngle += -180.0;
		}else if ((Swerve->DeltaAngle<-90)&&(Swerve->DeltaAngle>=-180))
		{
			Swerve->DeltaAngle += 180.0;
		}

		Swerve->PreAngle = InPut;
		Swerve->preCal = Swerve->CalInput;
		Swerve->CurrentAngle+= Swerve->DeltaAngle;


		if(Swerve->CalInput==modulo360(Swerve->CurrentAngle)){
			Swerve->Direc = 1;
		}
	}
}

