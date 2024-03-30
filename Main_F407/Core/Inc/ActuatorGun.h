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

/*-----------------------------Begin:PID GUN1 Macro(SPEED)----------------------*/
#define Gun1Proportion 			0.5
#define Gun1Integral			10
#define Gun1Derivatite			0
#define Gun1Alpha				0
#define Gun1DeltaT				0.005
#define Gun1SumAboveLimit 		1000
#define Gun1SumBelowLimit		-1000

/*-----------------------------End:PID GUN1 Macro(SPEED)------------------------*/

/*-----------------------------Begin:PID GUN2 Macro(SPEED)----------------------*/
#define Gun2Proportion 			0.8
#define Gun2Integral			10
#define Gun2Derivatite			0
#define Gun2Alpha				0
#define Gun2DeltaT				0.005
#define Gun2SumAboveLimit 		1000
#define Gun2SumBelowLimit		-1000

/*-----------------------------End:PID GUN2 Macro(SPEED)------------------------*/


#define DCEncoderPerRound 					1000
#define DCGearRatio 						3.535 // tỉ số truy�?n 2 puly 1/4

void gun_Init();
void gun_StartGetBall();
void gun_StopGetBall();
void gun_StartShootBall(uint16_t pwm);
void gun_StopShootBall();
void gun_StopAll();
void gun_VelCal(int gunCount1, int gunCount2);
void gun_PIDSpeed1(float Target1);
void gun_PIDSpeed2(float Target2);
void gun_PIDSetParam(PID_Param *pid,float kP,float kI,float kD,float alpha,float deltaT,float u_AboveLimit,float u_BelowLimit);
#endif /* INC_ACTUATORGUN_H_ */
