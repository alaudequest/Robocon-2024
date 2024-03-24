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
#include "PutBall.h"
#include "Encoder.h"

#define IMU_Wait		6
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

#define STATE_CHANGE_OFFSET_METER_X

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
typedef enum PD_Type{
	PD_X,
	PD_Y,
	PD_Theta,
}PD_Type;

typedef enum Signal_type{
	U,
	V,
	R,
	U_Control,
	V_Control,
	R_Control,
}Signal_type;

typedef enum Trajec_Type{
	TrajecX,
	TrajecY,
	TrajecTheta,
}Trajec_Type;

typedef struct processControl_Parameter{
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
	uint8_t ball;
	float Ball_dis;
	int floating_dis;
	float chasis_Vector_Speed;
	float chasis_Vector_TargetSpeed;
	uint8_t Process_Running;
	float Target_Head;
}processControl_Parameter;

float cal_absF(float num);
pd_Param PD_GetObjParam(PD_Type ID);
void PD_SetObjParam(PD_Type ID,pd_Param Param);

void PD_Enable(PD_Type ID);
void PD_Disable(PD_Type ID);

void process_Init();
void process_Run(uint8_t Run);

float process_GetCtrSignal(Signal_type ID);
void process_SetCTrSignal(Signal_type ID,float Sig);


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
void process_PD_TestX(float Target);
void process_PD_TestY(float Target);
void process_PD_TestTheta(float Target);
void process_PD_RotationControlSignal();
void process_RotationMatrix(float u, float v, float r);
void process_TrajecStateCondition_EndPath_NoYaw(float xCondition, float yCondition, float thetaCondition);

void process_setVal_PutBall(int Value);
void process_SetBallDis(float dis);
float GetXtraject(Trajec_Type ID);
uint8_t process_ReturnBallValue();
void process_GetBall();
void process_SetFloatingDis();
void process_ResetFloatingDis();
void process_Accel_FloatingEnc(float Angle,float maxSpeed,float s,float accel);
void process_GetBall2();
void process_WallApproach();
void process_GetBall3();
#endif /* INC_PROCESSCONTROL_H_ */
