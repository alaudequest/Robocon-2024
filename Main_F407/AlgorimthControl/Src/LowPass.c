/*
 * LowPass.c
 *
 *  Created on: Nov 5, 2023
 *      Author: Admin
 */

#include "LowPass.h"

#define filValue	Fil->filValue
#define preFilValue	Fil->preFilValue
#define filAlpha	Fil->filAlpha

float LowPassFilter(lowPassParam *Fil,float value)
{
	filValue = (1-filAlpha)*preFilValue+filAlpha*value;

	preFilValue = filValue;

	return filValue;
}
