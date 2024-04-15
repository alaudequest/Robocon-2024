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
static AccelerationState accelState = NO_ACCEL;
static float accelGunSpeed1, accelGunSpeed2, gunTargetSpeed1, gunTargetSpeed2;
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

void RB1_CalculateRuloGunPIDSpeed()
{
	RB1_VelocityCalculateOfGun();

	if (pidCurrentTickTimeGun1_ms >= Gun1DeltaT) {
		float uHat = PID_Calculate(&PID_Gun1, gunTargetSpeed1, encGun1.vel_Real);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, uHat);
		pidCurrentTickTimeGun1_ms = 0;
	}
	if (pidCurrentTickTimeGun2_ms >= Gun1DeltaT) {
		float uHat = PID_Calculate(&PID_Gun2, gunTargetSpeed2, encGun2.vel_Real);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, uHat);
		pidCurrentTickTimeGun2_ms = 0;
	}
}

void RB1_Gun_Start(bool enablePID, uint16_t targetPWM) {
	accelState = ACCELERATION;
}

void RB1_Gun_Stop() {
	accelState = DECELERATION;
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
			accelState = 0;
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
			accelState = 0;
		}
	}
}

void RB1_CollectBallMotor_ControlSpeed()
{
	if (accelState == 1) {
		CollectBallMotorSpeedUp();
	}
	else if (accelState == 2) {
		CollectBallMotorSpeedDown();
	}
	else {

	}
}

void RB1_CollectBallMotor_On()
{
	accelState = 1;
}

void RB1_CollectBallMotor_Off()
{
	accelState = 2;
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
