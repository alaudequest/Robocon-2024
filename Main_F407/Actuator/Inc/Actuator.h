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

#define HC595_NUM	1
#define VAN_NUM		4

typedef struct SensorInput_t{
	int32_t a;
}SensorInput_t;

typedef struct Motor_t{

}Motor_t;

typedef struct VanOutput_t{
	uint32_t currentVanOut;
	uint16_t pluseTime;
}VanOutput_t;

typedef enum {
	PROC_IDLE,
	PROC_START,
	BRD_VAN_ON,
	BRD_PLUSE_TIME,
	BRD_VAN_OFF,
	PROC_END,
}VanProcedure;

typedef struct ActuatorParam_t{
	SensorInput_t seninp;
	Motor_t mdc;
	VanOutput_t van;
	Encoder_t enc;
}ActuatorParam_t;

void actor_VanAssignPin();
void actor_SetVanOut(uint32_t van);
void actor_ClearVanOut(uint32_t van);
uint32_t actor_GetVanOut();
#endif /* INC_ACTUATOR_H_ */
