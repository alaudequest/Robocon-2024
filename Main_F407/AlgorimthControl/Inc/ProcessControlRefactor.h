/*
 * ProcessControlRefactor.h
 *
 *  Created on: Mar 17, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_PROCESSCONTROLREFACTOR_H_
#define INC_PROCESSCONTROLREFACTOR_H_
#include "stdbool.h"
typedef enum PID_Axis {
	PID_AxisX,
	PID_AxisY,
	PID_AxisTheta,
} PID_Axis;

typedef struct ProcessOutputResult {
	float uControl;
	float vControl;
	float rControl;
} ProcessOutputResult;

typedef uint8_t ManualSetStage;
typedef struct ManualSetParameters {
	float u;
	float v;
	float r;
	bool DisablePID_AxisX;
	bool DisablePID_AxisY;
	bool DisablePID_AxisTheta;
} ManualSetParameters;

typedef enum ProcessControlFlag {
	USE_PID_AXIS_X,
	USE_PID_AXIS_Y,
	USE_PID_AXIS_THETA,
	ON_STEADY_STATE,
	LOCK_SETTING_PARAM_TRAJECTORY,
} ProcessControlFlag;

typedef enum ControlType {
	ON_MANUAL_SET_CONTROL,
	ON_TRAJECTORY_PLANNING_CONTROL,
	ON_ACTUATOR,
} ControlType;

typedef struct AxisData {
	TrajectoryCalculateParameters trajectCalParams;
	PID_Param pid;
	bool enablePID;
} AxisData;

typedef struct AxesTrajectPoint {
	TrajectPlanningPoint trajectPointX;
	TrajectPlanningPoint trajectPointY;
	TrajectPlanningPoint trajectPointTheta;
	ControlType nextStageControlType;
} AxesTrajectPoint;

void process_PutTrajectPointToArray(AxesTrajectPoint p, TrajectoryStage stage);
AxesTrajectPoint process_GetTrajectPointFromArray(TrajectoryStage stage);
void proces_PutManualSetValueToArray(ManualSetParameters input, ManualSetStage stage);
ManualSetParameters proces_GetManualSetValueFromArray(ManualSetStage stage);
void SetAxisParamsPID(PID_Axis axis, PID_Param pid);
PID_Param GetAxisParamsPID(PID_Axis axis);

void process_Init();
void process_ManualSetChangeStage();
ProcessOutputResult process_Run(uint8_t Run);
float process_GetOutputValueOfPID(AxisData *axis, float odometerAxisPoseValue, float manualSetValue);
#endif /* INC_PROCESSCONTROLREFACTOR_H_ */
