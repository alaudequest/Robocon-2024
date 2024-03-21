/*
 * ProcessControlRefactor.h
 *
 *  Created on: Mar 17, 2024
 *      Author: SpiritBoi
 */

#ifndef INC_PROCESSCONTROLREFACTOR_H_
#define INC_PROCESSCONTROLREFACTOR_H_
#include "stdbool.h"
#include "PID.h"
#include "main.h"
#include "TrajectoryPlanning.h"

/**
 * Example code
Process_Init(){
	PID_Param pidInit;
	TrajectPlanningPoint tpInit;

	pidInit.kP = 1;
	pidInit.u_AboveLimit = 1;
	pidInit.u_BelowLimit = -1;
	pidInit.deltaT = 0.05;
	process_SetupAxisParameter(pidInit, tpInit, PID_AxisX);
	process_SetupAxisParameter(pidInit, tpInit, PID_AxisY);
	pidInit.kP = 1.2;
	process_SetupAxisParameter(pidInit, tpInit, PID_AxisTheta);
}

int main(){
	AxesTrajectPoint point;
	point.trajectPointX.pf = 1; // move X axis 1 meter
	point.trajectPointX.tf = 2; // in 2 second
	point.trajectPointX.vf = 0; // final velocity is 0 m/s
	point.trajectPointX.ReachOffset = 0.03; // offset is 0.03m

	point.trajectPointY.pf = 0;
	point.trajectPointY.tf = 2;
	point.trajectPointY.vf = 0;
	point.trajectPointY.ReachOffset = 0.03;

	point.trajectPointTheta.pf = 0;
	point.trajectPointTheta.tf = 2;
	point.trajectPointTheta.vf = 0;
	point.trajectPointTheta.ReachOffset = 0.03;
	point.nextStageControlType = ON_TRAJECTORY_PLANNING_CONTROL;

	process_PutTrajectPointToArray(point, 0);

	point.trajectPointX.pf = 1;
	point.trajectPointX.tf = 2;
	point.trajectPointX.vf = 0;
	point.trajectPointX.ReachOffset = 0.03;

	point.trajectPointY.pf = 1;
	point.trajectPointY.tf = 2;
	point.trajectPointY.vf = 0;
	point.trajectPointY.ReachOffset = 0.03;

	point.trajectPointTheta.pf = 0;
	point.trajectPointTheta.tf = 2;
	point.trajectPointTheta.vf = 0;
	point.trajectPointTheta.ReachOffset = 0.03;
	point.nextStageControlType = ON_MANUAL_SET_CONTROL;
	process_PutTrajectPointToArray(point1, 2);

	ManualSetParameters manualset;
	manualset.DisablePID_AxisTheta = 1;
	manualset.DisablePID_AxisX = 1;
	manualset.DisablePID_AxisY = 1;
	manualset.u = 0;
	manualset.v = 0;
	manualset.r = 0;
	point.nextStageControlType = NO_CONTROL;

	ProcessOutputResult result = process_Run(Run);
}
 */

/**
 * @brief PID for each axis X, Y and rotation Theta
 */
typedef enum PID_Axis {
	PID_AxisX,         /**< PID_AxisX */
	PID_AxisY,         /**< PID_AxisY */
	PID_AxisTheta, /**< PID_AxisTheta */
} PID_Axis;

typedef enum ControlType {
	NO_CONTROL, /**< NO_CONTROL */
	ON_MANUAL_SET_CONTROL, /**< ON_MANUAL_SET_CONTROL */
	ON_TRAJECTORY_PLANNING_CONTROL,/**< ON_TRAJECTORY_PLANNING_CONTROL */
	ON_ACTUATOR, /**< ON_ACTUATOR */
	ON_DELAY,
	WAIT_DIGITALSENSOR,
} ControlType;
/**
 * @brief The output u,v,r is the result of tranformation matrix
 * from robot coodinate to ground surface coordinate
 */
typedef struct ProcessOutputResult {
	float uControl;
	float vControl;
	float rControl;
} ProcessOutputResult;

typedef enum DigitalSensor {
	NOT_USE,
	SENSOR_FORKLIFT,
	SENSOR_CATCH_BALL,
	SENSOR_SILO,
} DigitalSensor;

typedef struct DigitalSensor_t {
	DigitalSensor sensor;
	ControlType nextControlType;
} DigitalSensor_t;

typedef uint8_t DelayStage;
typedef struct DelayParameter_t {
	ControlType nextControlType;
	uint16_t delay;
} DelayParameter_t;

typedef uint8_t ManualSetStage;

/**
 * @brief Bypass the result of PID calculate, use the user input u,v,r instead
 * to pass these values to transformation matrix
 */
typedef struct ManualSetParameters {
	float u;
	float v;
	float r;
	bool DisablePID_AxisX;
	bool DisablePID_AxisY;
	bool DisablePID_AxisTheta;
} ManualSetParameters;

/**
 * @brief Select control type for robot
 */


/**
 * @brief Data structure for each axis
 */
typedef struct AxisData {
	TrajectoryCalculateParameters trajectCalParams;
	PID_Param pid;
	bool enablePID;
} AxisData;

/**
 * @brief User input for each trajectory point of axes
 */
typedef struct AxesTrajectPoint {
	TrajectPlanningPoint trajectPointX;
	TrajectPlanningPoint trajectPointY;
	TrajectPlanningPoint trajectPointTheta;
	ControlType nextStageControlType;
} AxesTrajectPoint;

void process_PutTrajectPointToArray(AxesTrajectPoint p, TrajectoryStage stage);
AxesTrajectPoint process_GetTrajectPointFromArray(TrajectoryStage stage);
void process_PutManualSetValueToArray(ManualSetParameters input, ManualSetStage stage);
ManualSetParameters process_GetManualSetValueFromArray(ManualSetStage stage);
void process_SetAxisParamsPID(PID_Axis axis, PID_Param pid);
PID_Param process_GetAxisParamsPID(PID_Axis axis);
void process_SetupAxisParameter(PID_Param pidAxisInit, TrajectPlanningPoint tpInit, PID_Axis typeAxis);
void process_ManualSetChangeStage();
ProcessOutputResult process_Run(bool Run);
float process_GetOutputValueOfPID(AxisData *axis, float odometerAxisPoseValue, float manualSetValue);
#endif /* INC_PROCESSCONTROLREFACTOR_H_ */
