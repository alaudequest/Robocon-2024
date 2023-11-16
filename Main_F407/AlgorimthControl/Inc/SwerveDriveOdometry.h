/*
 * SwerveDriveOdometry.h
 *
 *  Created on: Nov 11, 2023
 *      Author: Admin
 */

#ifndef INC_SWERVEDRIVEODOMETRY_H_
#define INC_SWERVEDRIVEODOMETRY_H_

//#include "Pose2d.h"
//#include "Rotation2d.h"
#include "ForwardKinematic.h"

typedef struct SwerveDriveOdometry{
//	Pose2d m_poseMeters;
//	Rotation2d m_gyroOffset;
//	Rotation2d m_previousAngle;
	float currX;
	float currY;
	float currTheta;

	float preX;
	float preY;
	float preTheta;

	float deltaX;
	float deltaY;
	float deltaTheta;


	float OffsetGyro;

	float s;
	float c;
	float deltaT;

	float poseX;
	float poseY;
	float poseTheta;
}SwerveDriveOdometry;

void SwerveOdo_init();
void SwerveOdo_PoseCal(float u, float v, float theta);

#endif /* INC_SWERVEDRIVEODOMETRY_H_ */
