/*
 * ProcessControl.c
 *
 *  Created on: Feb 17, 2024
 *      Author: Admin
 */

#include "ProcessControl.h"
#include "cmsis_os.h"

process_Param process;
extern UART_HandleTypeDef huart1;

char tx_ResetBuff[] = "rst\n";
char tx_ReadBuff[]  = "red\n";

#define KP_INIT_X		1
#define KP_INIT_Y		1
#define KP_INIT_THETA	1.2

#define KD_INIT_X		0
#define KD_INIT_Y		0
#define KD_INIT_THETA	0

#define ALPHA_INIT_X	0
#define ALPHA_INIT_Y	0
#define ALPHA_INIT_THETA	0

#define LIMIT_ABOVE_INIT_X 		1
#define LIMIT_ABOVE_INIT_Y		1
#define LIMIT_ABOVE_INIT_THETA	1

#define LIMIT_BELOW_INIT_X 		-1
#define LIMIT_BELOW_INIT_Y		-1
#define LIMIT_BELOW_INIT_THETA	-1

#define U_APPROACH_START		0.05
#define V_APPROACH_START		-0.1
#define R_APPROACH_START		0.05

#define U_APPROACH_END		0.1
#define V_APPROACH_END		0
#define R_APPROACH_END		0

#define WAIT_APPROACH			2

#define WAIT_GETRICEPLANT		300

#define WAIT_SSCHECK	10
#define U_GETRICE		0
#define V_GETRICE		0
#define R_GETRICE		0

#define U_STEADY		0
#define V_STEADY		0
#define R_STEADY		0

#define THETA_STEADY	2*M_PI/180
#define X_STEADY		0.03
#define Y_STEADY		0.03

#define WAIT_STEADY		15

//-----------------------------------State 0 ---------------------------------------//
//-------Control Signal-------//
#define U_STATE0	0.1
#define V_STATE0	0
#define R_STATE0	0
//-------X POS-------//
#define PF_STATE0_X -0.20
#define P0_STATE0_X 0
#define TF_STATE0_X 1
#define V0_STATE0_X 0
#define VF_STATE0_X 0
//-------Y POS-------//
#define PF_STATE0_Y -0.25
#define P0_STATE0_Y 0
#define TF_STATE0_Y 1
#define V0_STATE0_Y 0
#define VF_STATE0_Y 0
//-------Y POS-------//
#define PF_STATE0_THETA 0
#define P0_STATE0_THETA 0
#define TF_STATE0_THETA 1
#define V0_STATE0_THETA 0
#define VF_STATE0_THETA 0

//-----------------------------------State 1 ---------------------------------------//
#define CONDITION_STATE1_X 0.03
#define CONDITION_STATE1_Y 0.03

//-----------------------------------State 2 ---------------------------------------//
//-------X POS-------//
#define PF_STATE2_X -0.25
#define TF_STATE2_X 2
#define VF_STATE2_X 0
//-------Y POS-------//
#define PF_STATE2_Y -0.90
#define TF_STATE2_Y 2
#define VF_STATE2_Y 0
//-------Y POS-------//
#define PF_STATE2_THETA 0
#define TF_STATE2_THETA 1
#define VF_STATE2_THETA 0

//-----------------------------------State 3 ---------------------------------------//
#define CONDITION_STATE3_X 0.03
#define CONDITION_STATE3_Y 0.03

//-----------------------------------State 4 ---------------------------------------//
//-------Control Signal-------//
#define U_STATE4	0.05
#define V_STATE4	-0.12
#define R_STATE4	0

//-----------------------------------State 8 ---------------------------------------//
//-------X POS-------//
#define PF_STATE8_X -0.7
#define TF_STATE8_X 1.5
#define VF_STATE8_X 0
//-------Y POS-------//
#define PF_STATE8_Y 0
#define TF_STATE8_Y 1
#define VF_STATE8_Y 0
//-------Y POS-------//
#define PF_STATE8_THETA 0
#define TF_STATE8_THETA 1
#define VF_STATE8_THETA 0

//-----------------------------------State 9 ---------------------------------------//
#define CONDITION_STATE9_X 0.03
#define CONDITION_STATE9_Y 0.03

//-----------------------------------State 10 ---------------------------------------//
//-------X POS-------//
#define PF_STATE10_X -0.95
#define TF_STATE10_X 3
#define VF_STATE10_X 0
//-------Y POS-------//
#define PF_STATE10_Y -0.70
#define TF_STATE10_Y 3
#define VF_STATE10_Y 0
//-------Y POS-------//
#define PF_STATE10_THETA -89*M_PI/180
#define TF_STATE10_THETA 3
#define VF_STATE10_THETA 0


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

float process_GetUControl(){return process.uControl;}
void  process_SetUControl(float uControl){process.uControl = uControl;}
void  process_SetU(float u){process.u = u;}

float process_GetVControl(){return process.vControl;}
void  process_SetVControl(float vControl){process.vControl = vControl;}
void  process_SetV(float v){process.v = v;}

float process_GetRControl(){return process.rControl;}
void  process_SetRControl(float rControl){process.rControl = rControl;}
void  process_SetR(float r){process.r = r;}

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
			PD_Disable(PD_X);
			PD_Disable(PD_Y);
			PD_Disable(PD_Theta);
			process_SetSignal(U_STATE0,V_STATE0,R_STATE0);
			process.stateChange ++;

			if(process.stateChange>5){
				process_SetSignal(0,V_STATE0,R_STATE0);
				odo_ResetPose();
				trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE0_X, TF_STATE0_X, odo_GetUout(), VF_STATE0_X);
				trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE0_Y, TF_STATE0_Y, odo_GetVout(), VF_STATE0_Y);
				trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE0_THETA, TF_STATE0_THETA, odo_GetRout(), VF_STATE0_THETA);

				PD_Enable(PD_X);
				PD_Enable(PD_Y);
				PD_Enable(PD_Theta);

				process.stateChange = 0;
				process_ChangeState();
			}
			break;
		case 1:
			process_TrajecStateCondition_OnPath(CONDITION_STATE1_X,CONDITION_STATE1_Y);
			break;
		case 2:
			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE2_X, TF_STATE2_X, odo_GetUout(), VF_STATE2_X);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE2_Y, TF_STATE2_Y, odo_GetVout(), VF_STATE2_Y);
			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE2_THETA, TF_STATE2_THETA, odo_GetRout(), VF_STATE2_THETA);

			PD_Enable(PD_X);
			PD_Enable(PD_Y);
			PD_Enable(PD_Theta);

			process_ChangeState();
			break;
		case 3:
			process_TrajecStateCondition_OnPath(CONDITION_STATE3_X,CONDITION_STATE3_Y);
			break;
		case 4:
			PD_Disable(PD_X);
			PD_Disable(PD_Y);
			PD_Disable(PD_Theta);

			process_SetSignal(U_STATE4, V_STATE4, R_STATE4);
			process_ChangeState();
			break;
		case 6:
			process_RicePlantApproach();
			break;
		case 8:

			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE8_X, TF_STATE8_X, odo_GetUout(), VF_STATE8_X);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE8_Y, TF_STATE8_Y, odo_GetVout(), VF_STATE8_Y);
			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE8_THETA, TF_STATE8_THETA, odo_GetRout(), VF_STATE8_THETA);

			PD_Enable(PD_X);
			PD_Enable(PD_Y);
			PD_Enable(PD_Theta);

			process_ChangeState();
			break;
		case 9:
			process_TrajecStateCondition_OnPath(CONDITION_STATE9_X,CONDITION_STATE9_Y);
			break;
		case 10:
			trajecPlan_SetParam(&process.trajecX, odo_GetPoseX(), PF_STATE10_X, TF_STATE10_X, odo_GetUout(), VF_STATE10_X);
			trajecPlan_SetParam(&process.trajecY, odo_GetPoseY(), PF_STATE10_Y, TF_STATE10_Y, odo_GetVout(), VF_STATE10_Y);
			trajecPlan_SetParam(&process.trajecTheta, odo_GetPoseTheta(), PF_STATE10_THETA, TF_STATE10_THETA, odo_GetRout(), VF_STATE10_THETA);

			PD_Enable(PD_X);
			PD_Enable(PD_Y);
			PD_Enable(PD_Theta);

			process_ChangeState();
			break;
		case 11:
			process_TrajecStateCondition_EndPath(X_STEADY,Y_STEADY,THETA_STEADY);
			break;
		default:
			break;
	}
}


void process_RunSSAndActuator(void (*ptnBreakProtectionCallBack)())
{
	switch (process.state) {
		case 5:
			process_SSCheck();
			break;
		case 7 :
			process_GetRicePlant(ptnBreakProtectionCallBack);
			break;
		case 12 :
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
