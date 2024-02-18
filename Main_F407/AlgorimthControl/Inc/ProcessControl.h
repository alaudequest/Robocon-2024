/*
 * ProcessControl.h
 *
 *  Created on: Feb 17, 2024
 *      Author: Admin
 */

#ifndef INC_PROCESSCONTROL_H_
#define INC_PROCESSCONTROL_H_

#include "Odometry.h"
#include "PositionControl.h"
#include "ActuatorValve.h"
#define IMU_Wait		6

typedef enum PD_Type{
	PD_X,
	PD_Y,
	PD_Theta,
}PD_Type;

typedef struct process_Param{
	pd_Param pdX,pdY,pdTheta;
	trajec_Param trajecX,trajecY,trajecTheta;

	int state;
	uint8_t stateChange;
	uint8_t stopUsePDX;
	uint8_t stopUsePDY;
	uint8_t stopUsePDTheta;
	uint8_t ssCheck,steadyCheck;

	float u,v,r;
	float uControl,vControl,rControl;
	float yaw;
}process_Param;

float cal_absF(float num);
pd_Param PD_GetObjParam(PD_Type ID);
void PD_SetObjParam(PD_Type ID,pd_Param Param);

void PD_Enable_X();
void PD_Enable_Y();
void PD_Enable_Theta();

void PD_Disable_X();
void PD_Disable_Y();
void PD_Disable_Theta();

void process_Init();
void process_Run(uint8_t Run);

float process_GetUControl();
void  process_SetUControl(float uControl);
void  process_SetU(float u);

float process_GetVControl();
void  process_SetVControl(float vControl);
void  process_SetV(float v);

float process_GetRControl();
void  process_SetRControl(float rControl);
void  process_SetR(float r);

void process_SSCheck();
void process_ReadIMU();
void process_ResetIMU();

void process_SetYaw(float currAng);
float process_GetYaw();

void process_SetSignal(float u,float v,float r);
void process_RunChassis();
void process_RunSSAndActuator(void (*ptnBreakProtectionCallBack)());
void process_RicePlantApproach();
void process_GetRicePlant(void (*ptnBreakProtectionCallBack)());
void process_ChangeState();
void process_TrajecStateCondition_OnPath(float xCondition,float yCondition);
void process_TrajecStateCondition_EndPath(float xCondition,float yCondition,float thetaCondition);

#endif /* INC_PROCESSCONTROL_H_ */
