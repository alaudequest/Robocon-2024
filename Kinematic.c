/*
 * Kinematic.c
 *
 *  Created on: Sep 14, 2023
 *      Author: defaultuser0
 */
#include "Kinematic.h"

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
//Ham tinh toan chia het cho 180
float modulo180(float Angle)
{
	int Result = (int)Angle/180;
	return Angle-Result*180;
}

//Ham tinh toan chia het cho 360
float modulo360(float Angle)
{
	int Result = (int)Angle/360;
	return Angle-Result*360;
}

//Ham Tim Goc Xoay Toi Uu nhat cho cum Swerve
void OptimizeAngle (Optimizer *Optimizer,float InPut)
{
	if (InPut != Optimizer->PreAngle){

		Optimizer->CalInput = InPut;
		if ((Optimizer->CurrentAngle>=0)&&Optimizer->CalInput<0)Optimizer->CalInput+=360;
		else if ((Optimizer->CurrentAngle<0)&&Optimizer->CalInput>0) Optimizer->CalInput-=360;

		Optimizer->DeltaCal = Optimizer->CalInput-Optimizer->preCal;
		if(Optimizer->DeltaCal>180)Optimizer->DeltaCal+=-360;
		else if(Optimizer->DeltaCal<-180)Optimizer->DeltaCal+=360;


		if ((Optimizer->DeltaCal>90)&&(Optimizer->DeltaCal<180))
		{
			Optimizer->Direc *= -1;
		}else if ((Optimizer->DeltaCal<-90)&&(Optimizer->DeltaCal>-180))
		{
			Optimizer->Direc *= -1;
		}else if(abs(Optimizer->DeltaCal)==180)
		{
			Optimizer->Direc *= -1;
		}

		Optimizer->DeltaAngle = Optimizer->CalInput-modulo360(Optimizer->CurrentAngle);

		if(Optimizer->DeltaAngle>180)Optimizer->DeltaAngle+=-360;
		else if(Optimizer->DeltaAngle<-180)Optimizer->DeltaAngle+=360;

		if((Optimizer->DeltaAngle<=90)&&(Optimizer->DeltaAngle>=-90))
		{
			Optimizer->DeltaAngle = Optimizer->DeltaAngle;
		}else if ((Optimizer->DeltaAngle>90)&&(Optimizer->DeltaAngle<=180))
		{
			Optimizer->DeltaAngle += -180.0;
		}else if ((Optimizer->DeltaAngle<-90)&&(Optimizer->DeltaAngle>=-180))
		{
			Optimizer->DeltaAngle += 180.0;
		}

		Optimizer->PreAngle = InPut;
		Optimizer->preCal = Optimizer->CalInput;
		Optimizer->CurrentAngle+= Optimizer->DeltaAngle;


		if(Optimizer->CalInput==modulo360(Optimizer->CurrentAngle)){
			Optimizer->Direc = 1;
		}
	}
}

//De Tim Ra 4 goc phan tu + cac Vecto giua cac goc phan tu
int8_t quadrantCheck(float X, float Y)
{
	if((X>0)&&(Y>0))return 1;
	else if ((X<0)&&(Y>0))return 2;
	else if ((X<0)&&(Y<0))return 3;
	else if ((X>0)&&(Y<0))return 4;
	else if ((X>0)&&(Y==0))return -1;
	else if ((X<0)&&(Y==0))return -2;
	else if ((X==0)&&(Y>0))return -3;
	else if ((X==0)&&(Y<0))return -4;
	else return 0;
}
//Ham tinh dong hoc nghich cho robot dua vao van toc chuyen vi theo cac phuong

void InverseKine(Kinematic *Swerve,Optimizer *Optimizer,float u,float v ,float r)
{
	Swerve->wheel_Vel_X = v;
	Swerve->wheel_Vel_Y = u - robot_Lenght*r;

	Swerve->currQuad = quadrantCheck(Swerve->wheel_Vel_X, Swerve->wheel_Vel_Y);
	if(Swerve->currQuad == 0) {
		OptimizeAngle(Optimizer, Swerve->preQuad);
	}
	else {
		OptimizeAngle(Optimizer, atan2(Swerve->wheel_Vel_Y,Swerve->wheel_Vel_X)*180/M_PI);
		Swerve->preQuad = atan2(Swerve->wheel_Vel_Y, Swerve->wheel_Vel_X)*180/M_PI;
	}
	Swerve->wheel_AngleVel = Optimizer->Direc*(1/robot_WheelR)*(sqrt(pow(Swerve->wheel_Vel_X,2)+pow(Swerve->wheel_Vel_Y,2)))/0.1047198;
}


