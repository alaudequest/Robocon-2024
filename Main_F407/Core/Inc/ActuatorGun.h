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
#include "stdbool.h"

/*-----------------------------Begin:PID GUN1 Macro(SPEED)----------------------*/
#define Gun1Proportion 			0.5
#define Gun1Integral			10
#define Gun1Derivatite			0
#define Gun1Alpha				0
#define Gun1DeltaT				0.01
#define Gun1SumAboveLimit 		600
#define Gun1SumBelowLimit		0

/*-----------------------------End:PID GUN1 Macro(SPEED)------------------------*/

/*-----------------------------Begin:PID GUN2 Macro(SPEED)----------------------*/
#define Gun2Proportion 			0.5
#define Gun2Integral			10
#define Gun2Derivatite			0
#define Gun2Alpha				0
#define Gun2DeltaT				0.01
#define Gun2SumAboveLimit 		600
#define Gun2SumBelowLimit		0

/*-----------------------------End:PID GUN2 Macro(SPEED)------------------------*/

#define DCEncoderPerRound 					200
#define DCGearRatio 						1

typedef enum AccelerationState {
	NO_ACCEL,
	ACCELERATION,
	DECELERATION,
} AccelerationState;

typedef struct Acceleration_t {
	float currentOutputValue;
	float valueStep;
	float targetValue;
	uint16_t accelTick_ms;
	uint8_t accelTimeStep_ms;
	uint8_t numStep;
	bool lockNumStep;
} Acceleration_t;

void RB1_Gun_Init();
void RB1_Gun_Start(float gun1TargetSpeed, float gun2TargetSpeed);
void RB1_Gun_Stop();

void RB1_SetTargetSpeedGun1(float targetSpeed);
void RB1_SetTargetSpeedGun2(float targetSpeed);
void RB1_EnableRuloShootBall();
void RB1_DisableRuloShootBall();
float RB1_GetSpeedRuloShootBall1();
float RB1_GetSpeedRuloShootBall2();

void RB1_CalculateRuloGunPIDSpeed();
void RB1_VelocityCalculateOfGun();
void RB1_GunIncreaseTickTimerInInterrupt();
void RB1_UpdateAccelTickInInterrupt();


void RB1_CollectBallMotor_Init();
void RB1_CollectBallMotor_ControlSpeed();
void RB1_CollectBallMotor_IdleSpeed();
void RB1_CollectBallMotor_On();
void RB1_CollectBallMotor_Off();

void RB1_CollectBallMotorOffForce();
void RB1_CollectBallMotorOnMax();

void RB1_EncGun1_IncreaseCount();
void RB1_EncGun1_DecreaseCount();
void RB1_EncGun2_IncreaseCount();
void RB1_EncGun2_DecreaseCount();
void RB1_EncoderGun_ResetCount();
#endif /* INC_ACTUATORGUN_H_ */
