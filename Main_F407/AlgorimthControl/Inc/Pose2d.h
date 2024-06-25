/*
 * Pose2d.h
 *
 *  Created on: Nov 10, 2023
 *      Author: Admin
 */

#ifndef INC_POSE2D_H_
#define INC_POSE2D_H_

#include "math.h"

typedef struct Translation2d {
	float m_x;
	float m_y;
}Translation2d;

void TranslationAsign(float,float);
#endif /* INC_POSE2D_H_ */
