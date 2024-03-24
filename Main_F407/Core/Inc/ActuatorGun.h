/*
 * ActuatorGun.h
 *
 *  Created on: Mar 24, 2024
 *      Author: namdhay
 */

#ifndef INC_ACTUATORGUN_H_
#define INC_ACTUATORGUN_H_

#include "main.h"
#include "Motor.h"
#include "Encoder.h"

void gun_Init();
void gun_StartGetBall();
void gun_StopGetBall();
void gun_StartShootBall(uint16_t pwm);
void gun_StopShootBall();
void gun_StopAll();
#endif /* INC_ACTUATORGUN_H_ */
