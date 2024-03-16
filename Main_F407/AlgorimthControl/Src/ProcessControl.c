/*
 * ProcessControl.c
 *
 *  Created on: Feb 17, 2024
 *      Author: Admin
 */

#include "ProcessControl.h"
#include "cmsis_os.h"


processControl_Parameter process;
extern UART_HandleTypeDef huart1;

char tx_ResetBuff[] = "rst\n";
char tx_ReadBuff[]  = "red\n";


float cal_absF(float num)
{
	if (num<0.0){
			return num*-1.0;
		}
		else
		return num;
}
pd_Param PD_GetObjParam(PD_Type ID)
{
	switch (ID) {
		case PD_X:
			return process.pdX;
			break;
		case PD_Y:
			return process.pdY;
			break;
		case PD_Theta:
			return process.pdTheta;
			break;
	}
	return process.pdX;
}
void PD_SetObjParam(PD_Type ID,pd_Param Param){
	switch (ID) {
		case PD_X:
			process.pdX = Param;
			break;
		case PD_Y:
			process.pdY = Param;
			break;
		case PD_Theta:
			process.pdTheta = Param;
			break;
	}
}

void process_Init(){
	process_ResetIMU();
	process.trajecX.tf = 0;
	process.trajecY.tf = 0;
	process.trajecTheta.tf = 0;

	PD_SetParameter(&process.pdX, KP_INIT_X, KD_INIT_X, ALPHA_INIT_X);
	PD_SetParameter(&process.pdY, KP_INIT_Y, KD_INIT_Y, ALPHA_INIT_Y);
	PD_SetParameter(&process.pdTheta, KP_INIT_THETA, KD_INIT_THETA, ALPHA_INIT_THETA);

	PD_SetSaturate(&process.pdX, LIMIT_ABOVE_INIT_X, LIMIT_BELOW_INIT_X);
	PD_SetSaturate(&process.pdY, LIMIT_ABOVE_INIT_Y, LIMIT_BELOW_INIT_Y);
	PD_SetSaturate(&process.pdTheta, LIMIT_ABOVE_INIT_THETA, LIMIT_BELOW_INIT_THETA);
}

float process_GetCtrSignal(Signal_type ID)
{
	switch (ID) {
		case U:
			return process.u;
			break;
		case V:
			return process.v;
			break;
		case R:
			return process.r;
			break;
		case U_Control:
			return process.uControl;
			break;
		case V_Control:
			return process.vControl;
			break;
		case R_Control:
			return process.rControl;
			break;
		default:
			break;
	}
	return 0;
}
void process_SetCtrSignal(Signal_type ID,float Sig)
{
	switch (ID) {
		case U:
			process.u = Sig;
			break;
		case V:
			process.v = Sig;
		case R:
			process.r = Sig;
		case U_Control:
			process.uControl = Sig;
		case V_Control:
			process.vControl = Sig;
		case R_Control:
			process.rControl = Sig;
		default:
			break;
	}
}

void process_ChangeState(){process.state+=1;}

void PD_Enable(PD_Type ID){
	switch (ID) {
		case PD_X:
			process.stopUsePDX = 0;
			break;
		case PD_Y:
			process.stopUsePDY = 0;
			break;
		case PD_Theta:
			process.stopUsePDTheta = 0;
			break;
		default:
			break;
	}
}

void PD_Disable(PD_Type ID){
	switch (ID) {
		case PD_X:
			process.stopUsePDX = 1;
			break;
		case PD_Y:
			process.stopUsePDY = 1;
			break;
		case PD_Theta:
			process.stopUsePDTheta = 1;
			break;
		default:
			break;
	}
}



void process_RicePlantApproach()
{
	process_SetSignal(U_APPROACH_START, V_APPROACH_START, R_APPROACH_START);
	process.stateChange ++;
	if(process.stateChange > WAIT_APPROACH)
	{
		process.stateChange = 0;
		process_SetSignal(U_APPROACH_END, V_APPROACH_END, R_APPROACH_END);
		process_ChangeState();
	}

}

void process_SetSignal(float u,float v,float r)
{
	process.u =	u;
	process.v =	v;
	process.r =	r;
}

void process_TrajecStateCondition_OnPath(float xCondition,float yCondition)
{
	if ((cal_absF(odo_GetPoseX()-process.trajecX.Pf)<xCondition)&&(cal_absF(odo_GetPoseY()-process.trajecY.Pf)<yCondition)){
		process_ChangeState();
	}
}
void process_TrajecStateCondition_EndPath_NoYaw(float xCondition, float yCondition, float thetaCondition){
	if(process.stateChange == 0)
	{
		if ((cal_absF(odo_GetPoseX()-process.trajecX.Pf)<xCondition)&&(cal_absF(odo_GetPoseY()-process.trajecY.Pf)<yCondition)){

			process.stateChange = 1;
		}
	}
	if (process.stateChange == 1)
	{
		PD_Disable(PD_X);
		PD_Disable(PD_Y);
		process_SetCtrSignal(U, 0);
		process_SetCtrSignal(V, 0);
		if((cal_absF(odo_GetPoseTheta() - process.trajecTheta.Pf)<=thetaCondition))
		{
			process.steadyCheck ++;
		}else{
			process.steadyCheck = 0;
		}
		if(process.steadyCheck>WAIT_STEADY)
		{
			process.stateChange = 0;
			PD_Disable(PD_Theta);
			process_ChangeState();
			process_SetCtrSignal(R, 0);
		}
	}

}
void process_TrajecStateCondition_EndPath(float xCondition,float yCondition,float thetaCondition)
{
	if(process.stateChange == 0)
	{
		if ((cal_absF(odo_GetPoseX()-process.trajecX.Pf)<xCondition)&&(cal_absF(odo_GetPoseY()-process.trajecY.Pf)<yCondition)){
			odo_SetPoseTheta(process.yaw);
			process.stateChange = 1;
		}
	}
	if (process.stateChange == 1)
	{
		PD_Disable(PD_X);
		PD_Disable(PD_Y);
		process_SetSignal(U_STEADY, V_STEADY, R_STEADY);
		if((cal_absF(process.yaw - PF_STATE10_THETA)<=thetaCondition))
		{
			process.steadyCheck ++;
		}else{
			process.steadyCheck = 0;
		}

		if(process.steadyCheck>WAIT_STEADY)
		{
			process.stateChange = 0;
			PD_Disable(PD_Theta);
			process_ChangeState();
		}
	}
}

void process_SSCheck()
{
	if (HAL_GPIO_ReadPin(SSLua1_GPIO_Port, SSLua1_Pin)&&HAL_GPIO_ReadPin(SSLua2_GPIO_Port, SSLua2_Pin)){
		process.ssCheck ++;
	}else {
		process.ssCheck = 0;
	}

	if(process.ssCheck>WAIT_SSCHECK)
	{
		process.ssCheck = 0;
		process_ChangeState();
	}
}
void process_ReadIMU()
{
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_ReadBuff, strlen(tx_ReadBuff), 100);
	osDelay(IMU_Wait);

}
void process_ResetIMU()
{
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_ResetBuff, strlen(tx_ResetBuff), 100);
	osDelay(IMU_Wait);
}

void process_SetYaw(float currAng){process.yaw = currAng*M_PI/180;}
float process_GetYaw(){return process.yaw;}

void process_GetRicePlant(void (*ptnBreakProtectionCallBack)())
{
	valve_BothCatch();
	odo_ResetPose();
	process_ResetIMU();
//	ptnBreakProtectionCallBack();
//	osDelay(WAIT_GETRICEPLANT);
	process_SetSignal(U_GETRICE, V_GETRICE, R_GETRICE);
	process_ChangeState();
}

void process_RunChassis()
{
	switch (process.state) {
		case 0:
//			PD_Disable(PD_X);
//			PD_Disable(PD_Y);
//			PD_Disable(PD_Theta);
//			process_SetSignal(U_STATE0,V_STATE0,R_STATE0);
//			process.stateChange ++;
//
//			if(process.stateChange>5){
//				process_SetSignal(0,V_STATE0,R_STATE0);
//				odo_ResetPose();
//				trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE0_X, TF_STATE0_X, odo_GetUout(), VF_STATE0_X);
//				trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE0_Y, TF_STATE0_Y, odo_GetVout(), VF_STATE0_Y);
//				trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE0_THETA, TF_STATE0_THETA, odo_GetRout(), VF_STATE0_THETA);
//
//				PD_Enable(PD_X);
//				PD_Enable(PD_Y);
//				PD_Enable(PD_Theta);
//
//				process.stateChange = 0;
//				process_ChangeState();
//			}
			odo_ResetPose()	;
			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), 1.4, 3, odo_GetUout(), 0);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), -1.4, 3, odo_GetVout(), 0);
			trajecPlan_SetParam(&process.trajecTheta, 0,0*M_PI/180, 2, odo_GetRout(), 0);
			process_ChangeState();
			break;
		case 1:
//			process_TrajecStateCondition_OnPath(CONDITION_STATE1_X,CONDITION_STATE1_Y);
//			process_TrajecStateCondition_EndPath_NoYaw(0.02, 0.02,2*M_PI/180);
			odo_SetPoseTheta(process.yaw);
			process_TrajecStateCondition_OnPath(0.4, 0.4);
			break;
		case 2:
//			odo_ResetPose()	;
//			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE2_X, TF_STATE2_X, odo_GetUout(), VF_STATE2_X);
//			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE2_Y, TF_STATE2_Y, odo_GetVout(), VF_STATE2_Y);
//			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE2_THETA, TF_STATE2_THETA, odo_GetRout(), VF_STATE2_THETA);
//

////
////			process_ChangeState();
//			odo_ResetTheta();
			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), 1.2, 2, odo_GetUout(), 0);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), -2.0,2, odo_GetVout(), 0);
			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), -0*M_PI/180, 2, odo_GetRout(), 0);
			process_ChangeState();

			break;
		case 3:
			odo_SetPoseTheta(process.yaw);
			process_TrajecStateCondition_OnPath(0.06, 0.06);

//			process_TrajecStateCondition_EndPath_NoYaw(0.02, 0.02,2*M_PI/180);

			break;
		case 4:
			process_setVal_PutBall(1);
			process_ChangeState();
			break;
		case 5:
			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), odo_GetPoseX(), 2, odo_GetUout(), 0);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), odo_GetPoseY()+0.8, 2, odo_GetVout(), 0);
			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), -45*M_PI/180, 2, odo_GetRout(), 0);
			process_ChangeState();

			break;
		case 6:
			odo_SetPoseTheta(process.yaw);
			process_TrajecStateCondition_OnPath(0.1, 0.1);
//			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE8_X, TF_STATE8_X, odo_GetUout(), VF_STATE8_X);
//			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE8_Y, TF_STATE8_Y, odo_GetVout(), VF_STATE8_Y);
//			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE8_THETA, TF_STATE8_THETA, odo_GetRout(), VF_STATE8_THETA);
//
//			PD_Enable(PD_X);
//			PD_Enable(PD_Y);
//			PD_Enable(PD_Theta);
//
//			process_ChangeState();
			break;
		case 7:
			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), odo_GetPoseX()+1.6, 3, odo_GetUout(), 0);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), odo_GetPoseY(), 3, odo_GetVout(), 0);
			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), -45*M_PI/180, 2, odo_GetRout(), 0);

//			PD_Enable(PD_X);
//			PD_Enable(PD_Y);
//			PD_Enable(PD_Theta);
//
			process_ChangeState();
			break;
		case 8:
			odo_SetPoseTheta(process.yaw);
			process_TrajecStateCondition_OnPath(0.05,0.05);
			break;
		case 9:
			PD_Disable(PD_X);
			PD_Disable(PD_Y);
			PD_Disable(PD_Theta);

			process_SetSignal(0.05,-0.15,0);
			break;
		case 10 :
			process_SetSignal(0,0,0);
			process_ChangeState();
			break;
		default:
			break;
	}
}


void process_RunSSAndActuator(void (*ptnBreakProtectionCallBack)())
{
	switch (process.state) {
		case 9 :
			if (HAL_GPIO_ReadPin(SSLua2_GPIO_Port, SSLua2_Pin))
			{
				process.ssCheck ++;
			}else {
				process.ssCheck = 0;
			}
			if(process.ssCheck>30){
				process_ChangeState();
				process_setVal_PutBall(2);
				process.ssCheck = 0;
			}

//			process_GetRicePlant(ptnBreakProtectionCallBack);
			break;
		case 11 :
			osDelay(4000);
			process_setVal_PutBall(0);
			break;
		default:
			break;
	}
}
void process_Run(uint8_t Run){
	odo_PosCal();
	if(Run == 1){
		process.trajecX.t += DELTA_T;
		process.trajecY.t += DELTA_T;
		process.trajecTheta.t += DELTA_T;

		process_RunChassis();

		trajecPlan_Cal(&process.trajecX);
		trajecPlan_Cal(&process.trajecY);
		trajecPlan_Cal(&process.trajecTheta);

		if(!process.stopUsePDX)
		{
			PD_Cal(&process.pdX, process.trajecX.xTrajec, odo_GetPoseX());
			process.u = process.pdX.u + process.trajecX.xdottraject;

		}
		if(!process.stopUsePDY)
		{
			PD_Cal(&process.pdY, process.trajecY.xTrajec, odo_GetPoseY());
			process.v = process.pdY.u + process.trajecY.xdottraject;
		}
		if(!process.stopUsePDTheta)
		{
			PD_Cal(&process.pdTheta, process.trajecTheta.xTrajec, odo_GetPoseTheta());
			process.r = process.pdTheta.u + process.trajecTheta.xdottraject;
		}

		process.uControl = process.u*cos(-odo_GetPoseTheta()) - process.v*sin(-odo_GetPoseTheta());
		process.vControl = process.u*sin(-odo_GetPoseTheta()) + process.v*cos(-odo_GetPoseTheta());
		process.rControl = process.r;
	}
}
void process_setVal_PutBall(int Value)
{
	process.ball = Value;
}

uint8_t process_ReturnBallValue(){
	return process.ball;
}
void process_PD_TestX(float Target){
	PD_Cal(&process.pdX, Target, odo_GetPoseX());
	process.u = process.pdX.u ;

}
void process_PD_TestY(float Target){
	PD_Cal(&process.pdY, Target, odo_GetPoseY());
	process.v = process.pdY.u ;
}
void process_PD_TestTheta(float Target){
	PD_Cal(&process.pdTheta, Target, odo_GetPoseTheta());
	process.r = process.pdTheta.u;
}
void process_PD_RotationControlSignal(){
	process.uControl = process.u*cos(-odo_GetPoseTheta()) - process.v*sin(-odo_GetPoseTheta());
	process.vControl = process.u*sin(-odo_GetPoseTheta()) + process.v*cos(-odo_GetPoseTheta());
	process.rControl = process.r;
}
void process_RotationMatrix(float u, float v, float r){
	process.uControl = u*cos(-odo_GetPoseTheta()) - v*sin(-odo_GetPoseTheta());
	process.vControl = u*sin(-odo_GetPoseTheta()) + v*cos(-odo_GetPoseTheta());
	process.rControl = r;
}
