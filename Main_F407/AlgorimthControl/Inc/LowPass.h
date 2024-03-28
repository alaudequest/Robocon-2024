/*
 * LowPass.h
 *
 *  Created on: Nov 5, 2023
 *      Author: Admin
 */

#ifndef INC_LOWPASS_H_
#define INC_LOWPASS_H_

typedef struct lowPassParam{
	float filValue;
	float preFilValue;
	float filAlpha;
}lowPassParam;

float LowPassFilter(lowPassParam *Fil,float value);

#endif /* INC_LOWPASS_H_ */
