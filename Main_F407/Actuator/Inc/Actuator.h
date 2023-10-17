/*
 * Actuator.h
 *
 *  Created on: Oct 17, 2023
 *      Author: KHOA
 */

#ifndef INC_ACTUATOR_H_
#define INC_ACTUATOR_H_
#include "main.h"
#include "74HC595.h"
#include "Encoder.h"

typedef struct SensorInput_t{
	int32_t a;
}SensorInput_t;

typedef struct Motor_t{

}Motor_t;
typedef struct VanOutput_t{

}VanOutput_t;
typedef struct ActuatorParam_t{
	SensorInput_t seninp;
	Motor_t mdc;
	VanOutput_t vanout;
	Encoder_t enc;
}ActuatorParam_t;

void actor_SetVanOut(uint32_t van);
uint32_t actor_GetVanOut();
#endif /* INC_ACTUATOR_H_ */
