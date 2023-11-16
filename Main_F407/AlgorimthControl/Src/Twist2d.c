/*
 * Twist2d.c
 *
 *  Created on: Nov 12, 2023
 *      Author: Admin
 */

#include "Twist2d.h"

#define Dx			twist->dx
#define Dy 			twist->dy
#define Dtheta		twist->dtheta
#define S			twist->s
#define C			twist->c

void Twist2dSetParam(Twist2d *twist,float dx,float dy,float dtheta)
{
	Dx = dx;
	Dy = dy;
	Dtheta = dtheta;
}



