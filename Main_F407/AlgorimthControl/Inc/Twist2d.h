/*
 * Twist2d.h
 *
 *  Created on: Nov 12, 2023
 *      Author: Admin
 */

#ifndef INC_TWIST2D_H_
#define INC_TWIST2D_H_

typedef struct Twist2d{
	float dx;
	float dy;
	float dtheta;
	float s;
	float c;
}Twist2d;

void Twist2dSetParam(Twist2d *twist,float dx,float dy,float dtheta);
#endif /* INC_TWIST2D_H_ */
