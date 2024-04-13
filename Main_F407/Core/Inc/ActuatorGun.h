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
#include "PID.h"

/*-----------------------------Begin:PID GUN1 Macro(SPEED)----------------------*/
#define Gun1Proportion 			0.5
#define Gun1Integral			10
#define Gun1Derivatite			0
#define Gun1Alpha				0
#define Gun1DeltaT				0.01
#define Gun1SumAboveLimit 		1000
#define Gun1SumBelowLimit		0

/*-----------------------------End:PID GUN1 Macro(SPEED)------------------------*/

/*-----------------------------Begin:PID GUN2 Macro(SPEED)----------------------*/
#define Gun2Proportion 			0.5
#define Gun2Integral			10
#define Gun2Derivatite			0
#define Gun2Alpha				0
#define Gun2DeltaT				0.01
#define Gun2SumAboveLimit 		1000
#define Gun2SumBelowLimit		0

/*-----------------------------End:PID GUN2 Macro(SPEED)------------------------*/

#define DCEncoderPerRound 					200
#define DCGearRatio 						1

void gun_Init();
void gun_StartGetBall();
void gun_StopGetBall();
void VelCal(Encoder_t *enc, int count_X1, uint32_t count_PerRevol, float deltaT);
void gun_StartShootBall(uint16_t pwm);
void gun_StopShootBall();
void gun_StopAll();
void gun_VelCal(int gunCount1, int gunCount2);
void gun_PIDSpeed1(float Target1);
void gun_PIDSpeed2(float Target2);
void gun_ResetEncoder(int *gunCount1, int *gunCount2);
void encoderGun_ResetCount(Encoder_t *enc, int *count_X1);
void gun_PIDSetParam(PID_Param *pid, float kP, float kI, float kD, float alpha, float deltaT, float u_AboveLimit, float u_BelowLimit);
void RB1_CollectBallMotor_Init();
void RB1_CollectBallMotor_ControlSpeed();
void RB1_CollectBallMotor_On();
void RB1_CollectBallMotor_Off();
#endif /* INC_ACTUATORGUN_H_ */
