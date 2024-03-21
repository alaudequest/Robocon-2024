/*
 * ProcessControlRefactor.c
 *
 *  Created on: Mar 17, 2024
 *      Author: Nguyen Hai Hoang
 *      Refactor by: SpiritBoi
 */

#include "ProcessControlRefactor.h"
#include "Odometry.h"
#include "cmsis_os.h"

#define MAX_TRAJECT_STAGE 5
#define MAX_MANUAL_SET_STAGE 10
#define MAX_DELAY_STAGE 3
//#define CONTROL_TYPE_INIT ON_TRAJECTORY_PLANNING_CONTROL
#define CONTROL_TYPE_INIT ON_MANUAL_SET_CONTROL


AxisData X, Y, Theta;

ManualSetStage manualStage = 0;
TrajectoryStage trajectStage = 0;
DelayStage delayStage = 0;

DelayParameter_t arrDelay[MAX_DELAY_STAGE] = {0};
AxesTrajectPoint arrTrajectPoints[MAX_TRAJECT_STAGE] = {0};
ManualSetParameters arrManualSet[MAX_MANUAL_SET_STAGE];

uint32_t currentDelay = 0;
ControlType currentControlType = CONTROL_TYPE_INIT;
bool LockSettingTrajectParam = false;
bool resetOdometer = true;

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

static bool trajectory_IsReachTargetPoint(TrajectoryStage stage)
{
	return ((cal_absF(odo_GetPoseX() - X.trajectCalParams.tp.pf) < arrTrajectPoints[stage].trajectPointX.ReachOffset)
			&& (cal_absF(odo_GetPoseY() - Y.trajectCalParams.tp.pf) < arrTrajectPoints[stage].trajectPointY.ReachOffset));
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
	trajectplan_Calculate(&X.trajectCalParams);
	trajectplan_Calculate(&Y.trajectCalParams);
	trajectplan_Calculate(&Theta.trajectCalParams);
	if(trajectory_IsReachTargetPoint(trajectStage)) {
		LockSettingTrajectParam = false; // to set new parameters
		currentControlType = arrTrajectPoints[trajectStage].nextStageControlType;
		if(trajectStage < MAX_TRAJECT_STAGE) trajectStage++;
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
		case ON_TRAJECTORY_PLANNING_CONTROL:
			TrajectoryManager();
		break;
		case ON_ACTUATOR:
			break;
		case ON_DELAY:
		break;
		default:
			break;
	}
}

float process_GetOutputValueOfPID(AxisData *axis, float odometerAxisPoseValue, float manualSetValue) {
	if(axis->enablePID == 0) return manualSetValue;
	PID_Calculate(&axis->pid, axis->trajectCalParams.xTrajec, odometerAxisPoseValue);
	return axis->pid.u + axis->trajectCalParams.xDotTraject;
}



ProcessOutputResult process_Run(bool Run) {
	odo_PosCal();
	ProcessOutputResult output;
	if(Run) {
		odo_ResetPose_2(&resetOdometer);
		RunControlType(currentControlType);
		float poseX = odo_GetPoseX();
		float poseY = odo_GetPoseY();
		float poseTheta = odo_GetPoseTheta();
		float u = process_GetOutputValueOfPID(&X, poseX, arrManualSet[manualStage].u);
		float v = process_GetOutputValueOfPID(&Y, poseY, arrManualSet[manualStage].v);
		float r = process_GetOutputValueOfPID(&Theta, poseTheta, arrManualSet[manualStage].r);
		output.uControl = u * cos(-poseTheta) - v * sin(-poseTheta);
		output.vControl = u * sin(-poseTheta) + v * cos(-poseTheta);
		output.rControl = r;
	}
	return output;
}

void process_SetupAxisParameter(PID_Param pidAxisInit, TrajectPlanningPoint tpInit, PID_Axis typeAxis) {
	AxisData *targetAxis = NULL;
	switch (typeAxis) {
		case PID_AxisX:
			targetAxis = &X;
		break;
		case PID_AxisY:
			targetAxis = &Y;
		break;
		case PID_AxisTheta:
			targetAxis = &Theta;
		break;
	}
	PID_SetParameters(&targetAxis->pid, pidAxisInit.kP, pidAxisInit.kI, pidAxisInit.kD, pidAxisInit.alpha);
	PID_SetSaturate(&targetAxis->pid, pidAxisInit.u_AboveLimit, pidAxisInit.u_BelowLimit);
	trajectplan_SetCalculateParameters(&targetAxis->trajectCalParams, 0, 0, tpInit);
	targetAxis->pid.deltaT = pidAxisInit.deltaT;
}

void process_ManualSetChangeStage() {
	if(manualStage < MAX_MANUAL_SET_STAGE) manualStage++;
}

void process_PutManualSetValueToArray(ManualSetParameters input, ManualSetStage stage) {
	arrManualSet[stage] = input;
}

ManualSetParameters process_GetManualSetValueFromArray(ManualSetStage stage) {
	return arrManualSet[stage];
}

void process_PutTrajectPointToArray(AxesTrajectPoint p, TrajectoryStage stage) {
	arrTrajectPoints[stage] = p;
}

AxesTrajectPoint process_GetTrajectPointFromArray(TrajectoryStage stage) {
	return arrTrajectPoints[stage];
}

PID_Param process_GetAxisParamsPID(PID_Axis axis)
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

void process_SetAxisParamsPID(PID_Axis axis, PID_Param pid) {
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

void process_PutDelayValueToArray(uint32_t milisecond, uint8_t stage, ControlType nextControlType) {
	if(stage > MAX_DELAY_STAGE) return;
	arrDelay[stage].delay = milisecond;
	arrDelay[stage].nextControlType = nextControlType;
}

uint32_t process_GetDelayValueFromArray(uint8_t stage) {
	return arrDelay[stage].delay;
}

/**
 * Should be place in HAL_TIM_PeriodElapsedCallback
 */
void process_DelayInISR() {
	if(currentControlType != ON_DELAY) return;
	if(currentDelay < arrDelay[delayStage].delay)
		currentDelay++;
	else if(delayStage < MAX_DELAY_STAGE) {
		currentDelay = 0;
		delayStage++;
	}

}

