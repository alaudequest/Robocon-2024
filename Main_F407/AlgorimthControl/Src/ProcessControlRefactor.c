/*
 * ProcessControlRefactor.c
 *
 *  Created on: Mar 17, 2024
 *      Author: Nguyen Hai Hoang
 *      Refactor by: SpiritBoi
 */

#include "ProcessControlRefactor.h"
#include "TrajectoryPlanning.h"
#include "PID.h"
#include "Odometry.h"

#define MAX_TRAJECT_STAGE 10
#define MAX_MANUAL_SET_STAGE 10
#define CONTROL_TYPE_INIT ON_TRAJECTORY_PLANNING_CONTROL


AxisData X, Y, Theta;
ManualSetStage manualStage = 0;
TrajectoryStage trajectStage = 0;
ControlType currentControlType = CONTROL_TYPE_INIT;
AxesTrajectPoint arrTrajectPoints[MAX_TRAJECT_STAGE] = {0};
ManualSetParameters arrManualSet[MAX_MANUAL_SET_STAGE];
bool LockSettingTrajectParam = false;

static float cal_absF(float num)
{
	if(num < 0.0) {
		return num * -1.0;
	}
	else
		return num;
}

static void PID_Enable(PID_Axis axis) {
	switch (axis) {
		case PID_AxisX:
			X.enablePID = true;
		break;
		case PID_AxisY:
			Y.enablePID = true;
		break;
		case PID_AxisTheta:
			Theta.enablePID = true;
		break;
		default:
			break;
	}
}

static void PID_Disable(PID_Axis axis) {
	switch (axis) {
		case PID_AxisX:
			X.enablePID = false;
		break;
		case PID_AxisY:
			Y.enablePID = false;
		break;
		case PID_AxisTheta:
			Theta.enablePID = false;
		break;
		default:
			break;
	}
}

static bool trajectory_IsReachTargetPoint()
{
	return ((cal_absF(odo_GetPoseX() - X.trajectCalParams.pf) < X.trajectPoint.ReachOffset)
			&& (cal_absF(odo_GetPoseY() - Y.trajectCalParams.pf) < Y.trajectPoint.ReachOffset));
}

static void SetTrajectoryPlanningForNewStage(TrajectoryStage stage) {
	if(LockSettingTrajectParam != 0) return;
	trajectplan_SetCalculateParameters(&X.trajectCalParams, odo_GetPoseX(), odo_GetUout(), arrTrajectPoints[stage].trajectPointX);
	trajectplan_SetCalculateParameters(&Y.trajectCalParams, odo_GetPoseY(), odo_GetVout(), arrTrajectPoints[stage].trajectPointY);
	trajectplan_SetCalculateParameters(&Theta.trajectCalParams, odo_GetPoseTheta(), odo_GetRout(), arrTrajectPoints[stage].trajectPointTheta);
	PID_Enable(PID_AxisX);
	PID_Enable(PID_AxisY);
	PID_Enable(PID_AxisTheta);
	LockSettingTrajectParam = true; // lock these parameters for specific stage
}

static void TrajectoryManager() {
	SetTrajectoryPlanningForNewStage(trajectStage);
	X.trajectCalParams.t += DELTA_T;
	Y.trajectCalParams.t += DELTA_T;
	Theta.trajectCalParams.t += DELTA_T;
	trajecplan_Calculate(&X.trajectCalParams);
	trajecplan_Calculate(&Y.trajectCalParams);
	trajecplan_Calculate(&Theta.trajectCalParams);
	if(trajectory_IsReachTargetPoint()) {
		LockSettingTrajectParam = false; // to set new parameters
		if(trajectStage < MAX_TRAJECT_STAGE) trajectStage++;
		currentControlType = arrTrajectPoints[trajectStage].nextStageControlType;
	}
}

static void ManualSet_DisablePID(ManualSetStage stage) {
	if(arrManualSet[stage].DisablePID_AxisX) PID_Disable(PID_AxisX);
	if(arrManualSet[stage].DisablePID_AxisY) PID_Disable(PID_AxisY);
	if(arrManualSet[stage].DisablePID_AxisTheta) PID_Disable(PID_AxisTheta);
}

static void RunControlType(ControlType type) {
	switch (type) {
		case ON_MANUAL_SET_CONTROL:
			ManualSet_DisablePID(manualStage);
			break;
		case ON_TRAJECTORY_PLANNING:
			TrajectoryManager();
		break;
		case ON_ACTUATOR:
			break;
	}
}

float process_GetOutputValueOfPID(AxisData *axis, float odometerAxisPoseValue, float manualSetValue) {
	if(axis->enablePID == 0) return manualSetValue;
	PID_Calculate(axis->pid, axis->trajectCalParams.xTrajec, odometerAxisPoseValue);
	return axis->pid.u + axis->trajectCalParams.xDotTraject;
}



ProcessOutputResult process_Run(uint8_t Run) {
	odo_PosCal();
	if(Run == 0) return;
	ProcessOutputResult output;
	RunControlType(currentControlType);
	float poseX = odo_GetPoseX();
	float poseY = odo_GetPoseY();
	float poseTheta = odo_GetPoseTheta();
	float u = process_GetOutputValueOfPID(&X, poseX, arrManualSet[manualStage].u);
	float v = process_GetOutputValueOfPID(&Y, poseY, arrManualSet[manualStage].v);
	float r = process_GetOutputValueOfPID(&Theta, poseTheta, arrManualSet[manualStage].r);
	// Rotation matrix transformation from robot coordinate to the ground surface coordinate
	output.uControl = u * cos(-poseTheta) - v * sin(-poseTheta);
	output.vControl = u * sin(-poseTheta) + v * cos(-poseTheta);
	output.rControl = r;
	return output;
}

void process_Init() {
	X.trajectCalParams.tp.tf = 0;
	Y.trajectCalParams.tp.tf = 0;
	Theta.trajectCalParams.tp.tf = 0;
	PID_SetParameters(X.pid, 1, 0, 0, 0);
	PID_SetParameters(Y.pid, 1, 0, 0, 0);
	PID_SetParameters(Theta.pid, 1.2, 0, 0, 0);

	PID_SetSaturate(X.pid, 1, -1);
	PID_SetSaturate(Y.pid, 1, -1);
	PID_SetSaturate(Theta.pid, 1, -1);
}

void process_ManualSetChangeStage() {
	if(manualStage < MAX_MANUAL_SET_STAGE) manualStage++;
}

void proces_PutManualSetValueToArray(ManualSetParameters input, ManualSetStage stage) {
	arrManualSet[stage] = input;
}

ManualSetParameters proces_GetManualSetValueFromArray(ManualSetStage stage) {
	return arrManualSet[stage];
}

void proces_PutTrajectPointToArray(AxesTrajectPoint p, TrajectoryStage stage) {
	arrTrajectPoints[stage] = p;
}

AxesTrajectPoint proces_GetTrajectPointFromArray(TrajectoryStage stage) {
	return arrTrajectPoints[stage];
}

PID_Param GetAxisParamsPID(PID_Axis axis)
{
	switch (axis) {
		case PID_AxisX:
			return X.pid;
		break;
		case PID_AxisY:
			return Y.pid;
		break;
		case PID_AxisTheta:
			return Theta.pid;
		break;
	}
	return X.pid;
}

void SetAxisParamsPID(PID_Axis axis, PID_Param pid) {
	switch (axis) {
		case PID_AxisX:
			X.pid = pid;
		break;
		case PID_AxisY:
			Y.pid = pid;
		break;
		case PID_AxisTheta:
			Theta.pid = pid;
		break;
	}
}
