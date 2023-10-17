/*
 * Actuator.c
 *
 *  Created on: Oct 17, 2023
 *      Author: KHOA
 */

uint32_t vanOut;

#include "Actuator.h"

void actor_SetVanOut(uint32_t van){vanOut = van;}
uint32_t actor_GetVanOut(){return vanOut;}
