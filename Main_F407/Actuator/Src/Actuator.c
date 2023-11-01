/*
 * Actuator.c
 *
 *  Created on: Oct 17, 2023
 *      Author: KHOA
 */


#include "Actuator.h"

HC595 *hc595 = NULL;
ActuatorParam_t	actorParam;
VanProcedure vanProcState;

void actor_VanAssignPin(){
	HC595_SetTarget(hc595);
	HC595_AssignPin(hc595, HC595_LATCH_GPIO_Port, HC595_LATCH_Pin, HC595_LATCH);
	HC595_AssignPin(hc595, HC595_DS_GPIO_Port, HC595_DS_Pin, HC595_DS);
	HC595_AssignPin(hc595, HC595_CLK_GPIO_Port, HC595_CLK_Pin, HC595_CLK);
	HC595_AssignPin(hc595, HC595_OE_GPIO_Port, HC595_OE_Pin, HC595_OE);
}

void actor_SetVanOut(uint32_t van){
	if(van > VAN_NUM)	return;
	HC595_SetByteOutput(van);
	HC595_ShiftOut(NULL, HC595_NUM, 1);
}

void actor_ClearVanOut(uint32_t van){
	if(van > VAN_NUM)	return;
	HC595_ClearByteOutput(van);
	HC595_ShiftOut(NULL, HC595_NUM, 1);
}

void actor_Procedure_Trigger_Van(){
//	static uint16_t pluseTime;
	static uint32_t vanToTrigger;
	switch(vanProcState){
		case PROC_IDLE:
			break;
		case PROC_START:
			break;
		case BRD_VAN_ON:
			actor_SetVanOut(vanToTrigger);
			break;
		case BRD_PLUSE_TIME:
			break;
		case BRD_VAN_OFF:
			actor_ClearVanOut(vanToTrigger);
			break;
		case PROC_END:
			vanProcState = PROC_IDLE;
			break;
	}
}

uint32_t actor_GetVanOut(){return actorParam.van.currentVanOut;}
uint16_t actor_GetPluseTime(){return actorParam.van.pluseTime;}
