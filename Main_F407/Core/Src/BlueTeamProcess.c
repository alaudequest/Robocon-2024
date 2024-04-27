///*
// * BlueTeamProcess.c
// *
// *  Created on: Apr 27, 2024
// *      Author: Admin
// */
//
//#include "BlueTeamProcess.h"
//extern trajec_Param trajecTheta;
//extern uint8_t Run;
//extern uint8_t step;
//extern float AngleNow;
//extern int floatingEncCount;
//extern uint8_t process_SubState;
//extern uint8_t process_SSCheck;
//extern float a_Now;
//extern int column,row;
//
//void BlueTeamRunProcess()
//{
//	if(step == 2)
//	{
//		process_setVal_PutBall(1);
//		AngleNow = -133;
//		process_PD_Critical();
//		process_Accel_FloatingEnc3(-133, 1.2, 10000, 0.3, 0, 3,5);
//		if(floatingEncCount > 8500)
//		{
//			step += 1;
//			process_SubState = 0;
//		}
//	}
//	else if (step == 3)
//	{
//
//		process_Accel_FloatingEnc3(-140, 1.2, 10000, 0.1, 0, 3,5);
//		if(floatingEncCount > 2000)
//		{
//			Reset_MPU_Angle();
//		}
//		if(floatingEncCount > 3600)
//		{
//			step += 1;
//			process_SubState = 0;
//		}
//	}
//	else if (step == 4)
//	{
//		process_Accel_FloatingEnc3(-45, 1.2, 20000, 0.1, 0, 3,10);
//		if(floatingEncCount > 7600)
//		{
//			step += 1;
//			process_SubState = 0;
//			process_ResetFloatingEnc();
//		}
//	}
//	else if (step == 5)
//	{
//		process_Accel_FloatingEnc3(-135, 0.8, 10000, 0.1, 0, 2.5,5);
//
//
//			if (floatingEncCount > 2500){
//			if(!HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin) )
//			{
//				process_SSCheck++;
//			}else{
//				process_SSCheck = 0;
//			}
//			if (process_SSCheck>1){
//				step += 1;
//				process_SubState = 0;
//			}
//		}
//
//	}
//	else if (step == 6)
//	{
//		process_Accel_FloatingEnc3(-135, 1, 50, 0.1, 0, 2.5,5);
//
//	}
//	else if (step == 7)
//	{
//		process_PD_OnStrainghtPath();
//		process_Accel_FloatingEnc4(-135, 1, 0, 0.1, 90, 2.5);
//		if(abs(a_Now/10 - 90)<10){
//			process_SubState = 0;
//			step++;
//		}
//	}
//	else if (step == 8)
//	{
//
//		process_Accel_FloatingEnc3(-230, 1, 10000, 0.1,90,2.5,5);
//		if(HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin))
//		{
//			process_SSCheck++;
//		}else{
//			process_SSCheck = 0;
//		}
//		if (process_SSCheck>10){
//			step += 1;
//			process_SubState = 0;
//		}
//	}
//	else if (step == 9)
//	{
//
//		process_Accel_FloatingEnc3(-230, 0.8, 5900, 0.1,90,2.5,5);
//	}
//	else if (step == 10)
//	{
//		Reset_MPU_Angle();
//		process_setVal_PutBall(1);
//		trajecPlan_Reset(&trajecTheta);
//		osDelay(1000);
//		process_PD_OnStrainghtPath();
//		step++;
//	}
//	else if (step == 11)
//	{
//
//		AngleNow = -90;
//		if (floatingEncCount<1600)
//		{
//			process_Accel_FloatingEnc3(-90, 0.6, 10000, 0.1,0,2.5,5);
//		}
//
//		if ((floatingEncCount>1600)&&(floatingEncCount<1800))
//		{
//			process_Accel_FloatingEnc3(-90, 0.25, 10000, 0.1,0,2.5,5);
//		}
//
//		if (floatingEncCount>1800)
//		{
//			step = 12;
//			process_SubState = 0;
//		}
//	}
//	else if (step == 12)
//	{
//
//			process_Accel_FloatingEnc3(0, 0.25, 10000, 0.1,0,2.5,10);
//
//		if (floatingEncCount>500)
//		{
//			step = 13;
//			process_SubState = 0;
//		}
//	}
//
//	else if (step == 13)
//	{
//		process_Ball_Approach3(column);
//
//	}
//	else if (step == 14)
//	{
//		process_getBall();
//	}
//}
//
