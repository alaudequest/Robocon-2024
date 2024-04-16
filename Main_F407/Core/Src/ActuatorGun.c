/*
 * ActuatorGun.c
 *
 *  Created on: Mar 24, 2024
 *      Author: namdhay
 */

#include "ActuatorGun.h"
#include "PID.h"
#include "Encoder.h"
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
static Encoder_t encGun1;
static Encoder_t encGun2;
static PID_Param PID_Gun1;
static PID_Param PID_Gun2;

static uint16_t collectBallPWM = 0;
static uint32_t collectBallTickTime = 0;
static uint8_t pidCurrentTickTimeGun1_ms = 0;
static uint8_t pidCurrentTickTimeGun2_ms = 0;
static uint8_t accelTickGun1 = 0;
static uint8_t accelTickGun2 = 0;
static AccelerationState accelStateCollectBall = NO_ACCEL;
static AccelerationState accelStateGun = NO_ACCEL;
static float speedInAccelGun1, speedInAccelGun2, gunTargetSpeed1, gunTargetSpeed2;

#define ACCEL_TIME_STEP (0.01 * 1000 * 15) // GunDeltaT * 1000ms * 15 = 0.01 * 1000 * 15 = 150ms

void RB1_Gun_Init() {
	// Start MOTOR GUN INIT
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	PID_SetParameters(&PID_Gun1, Gun1Proportion, Gun1Integral, Gun1Derivatite, Gun1Alpha);
	PID_SetSaturate(&PID_Gun1, Gun1SumAboveLimit, Gun1SumBelowLimit);
	encGun1.deltaT = PID_Gun1.deltaT = Gun1DeltaT;

	PID_SetParameters(&PID_Gun2, Gun2Proportion, Gun2Integral, Gun2Derivatite, Gun2Alpha);
	PID_SetSaturate(&PID_Gun2, Gun2SumAboveLimit, Gun2SumBelowLimit);
	encGun2.deltaT = PID_Gun2.deltaT = Gun2DeltaT;
	// End MOTOR GUN INIT

	// Start MOTOR RULO GET BALL INIT

	// End MOTOR RULO GET BALL INIT
}

static void VelocityCalculate(Encoder_t *enc)
{

	enc->vel_Real = ((enc->count_X1 - enc->count_Pre) / enc->deltaT) / (DCEncoderPerRound) * 60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real + 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_Pre = enc->count_X1;
}

void RB1_VelocityCalculateOfGun()
{
	VelocityCalculate(&encGun1);
	VelocityCalculate(&encGun2);
}

void RB1_UpdateAccelTickInInterrupt()
{
	if (accelStateGun == ACCELERATION || accelStateGun == DECELERATION) {
		accelTickGun1++;
		accelTickGun2++;
	}
}

static void CalculateAccelValue(AccelerationState *accelState, uint8_t *accelTick, uint8_t accelTimeStep, float *valueInAccel, float targetValue, uint8_t numStep)
{
	if (*accelTick < accelTimeStep)
		return;
	float valueStep = targetValue / numStep;
	if (*accelState == ACCELERATION) {
		*valueInAccel += valueStep;
		if (targetValue - *valueInAccel < valueStep) {
			*valueInAccel = targetValue;
			*accelState = NO_ACCEL;
		}
	}
	else if (*accelState == DECELERATION) {
		*valueInAccel -= valueStep;
		if (*valueInAccel - targetValue < valueStep) {
			*valueInAccel = targetValue;
			*accelState = NO_ACCEL;
		}
	}
	*accelTick = 0;
}

void RB1_CalculateRuloGunPIDSpeed()
{
	float targetSpeed1, targetSpeed2;
	RB1_VelocityCalculateOfGun();
	if (accelStateGun == ACCELERATION || accelStateGun == DECELERATION) {
		CalculateAccelValue(&accelStateGun, &accelTickGun1, ACCEL_TIME_STEP, &speedInAccelGun1, gunTargetSpeed1, 10);
		targetSpeed1 = speedInAccelGun1;
		CalculateAccelValue(&accelStateGun, &accelTickGun2, ACCEL_TIME_STEP, &speedInAccelGun2, gunTargetSpeed2, 10);
		targetSpeed2 = speedInAccelGun2;
	}
	else {
		targetSpeed1 = gunTargetSpeed1;
		targetSpeed2 = gunTargetSpeed2;
	}
	if (pidCurrentTickTimeGun1_ms >= Gun1DeltaT * 1000) { // DeltaT = 0.01s = 10ms
		float uHat = PID_Calculate(&PID_Gun1, targetSpeed1, encGun1.vel_Real);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, uHat);
		pidCurrentTickTimeGun1_ms = 0;
	}
	if (pidCurrentTickTimeGun2_ms >= Gun2DeltaT * 1000) {
		float uHat = PID_Calculate(&PID_Gun2, targetSpeed2, encGun2.vel_Real);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, uHat);
		pidCurrentTickTimeGun2_ms = 0;
	}
}

void RB1_Gun_Start() {
	accelStateGun = ACCELERATION;
	accelTickGun1 = 0;
	accelTickGun2 = 0;
}

void RB1_Gun_Stop() {
	accelStateGun = DECELERATION;
	accelTickGun1 = 0;
	accelTickGun2 = 0;
}

void RB1_SetTargetSpeedGun1(float targetSpeed) {
	gunTargetSpeed1 = targetSpeed;
}

void RB1_SetTargetSpeedGun2(float targetSpeed) {
	gunTargetSpeed2 = targetSpeed;
}

#define COLLECT_BALL_ACCEL_TIME_STEP 150
#define COLLECT_BALL_MAX_SPEED 1000

void RB1_CollectBallMotor_Init()
{
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

static void CollectBallMotorSpeedUp()
{
	if (HAL_GetTick() - collectBallTickTime > COLLECT_BALL_ACCEL_TIME_STEP && collectBallPWM <= COLLECT_BALL_MAX_SPEED) {
		collectBallTickTime = HAL_GetTick();
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, collectBallPWM);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, collectBallPWM);
		collectBallPWM += 100;

		if (collectBallPWM > 1000) {
			accelStateCollectBall = 0;
		}
	}

}

static void CollectBallMotorSpeedDown()
{
	if (HAL_GetTick() - collectBallTickTime > 150 && collectBallPWM >= 0) {
		collectBallTickTime = HAL_GetTick();
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, collectBallPWM);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, collectBallPWM);
		if (collectBallPWM < 100)
			collectBallPWM = 100;
		else
			collectBallPWM -= 100;

		if (collectBallPWM <= 0) {
			accelStateCollectBall = 0;
		}
	}
}

void RB1_CollectBallMotor_ControlSpeed()
{
	if (accelStateCollectBall != ACCELERATION || accelStateCollectBall != DECELERATION)
		return;
	if (accelStateCollectBall == ACCELERATION) {
		CollectBallMotorSpeedUp();
	}
	else if (accelStateCollectBall == DECELERATION) {
		CollectBallMotorSpeedDown();
	}
}

void RB1_CollectBallMotor_On()
{
	accelStateCollectBall = ACCELERATION;
}

void RB1_CollectBallMotor_Off()
{
	accelStateCollectBall = DECELERATION;
}

void RB1_GunIncreaseTickTimerInInterrupt()
{
	pidCurrentTickTimeGun1_ms++;
	pidCurrentTickTimeGun2_ms++;
}

static void EncoderResetCount(Encoder_t *enc)
{
	enc->vel_Real = 0;
	enc->vel_Pre = 0;
	enc->count_X1 = 0;
	enc->count_X4 = 0;
}

void RB1_EncGun1_IncreaseCount()
{
	encGun1.count_X1++;
}

void RB1_EncGun1_DecreaseCount()
{
	encGun1.count_X1--;
}

void RB1_EncGun2_IncreaseCount()
{
	encGun2.count_X1++;
}

void RB1_EncGun2_DecreaseCount()
{
	encGun2.count_X1++;
}

void RB1_EncoderGun_ResetCount()
{
	EncoderResetCount(&encGun1);
	EncoderResetCount(&encGun2);
}
