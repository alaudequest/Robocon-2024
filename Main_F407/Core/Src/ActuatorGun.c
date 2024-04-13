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
Encoder_t ENC_Gun1;
Encoder_t ENC_Gun2;
PID_Param PID_Gun1;
PID_Param PID_Gun2;

uint16_t collectBallPWM = 0;
uint32_t collectBallTickTime = 0;
uint8_t accelState = 0;

void gun_PIDSetParam(PID_Param *pid, float kP, float kI, float kD, float alpha, float deltaT, float u_AboveLimit, float u_BelowLimit)
{
//----------------------Term-----------------------//
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->alpha = alpha;
	pid->kB = 1 / deltaT;
//----------------------Sample Time----------------//
	pid->deltaT = deltaT;
//----------------------Limit----------------------//
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}

void gun_Init() {
	// Start MOTOR GUN INIT
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	gun_PIDSetParam(&PID_Gun1, Gun1Proportion, Gun1Integral, Gun1Derivatite, Gun1Alpha, Gun1DeltaT, Gun1SumAboveLimit, Gun1SumBelowLimit);
	gun_PIDSetParam(&PID_Gun2, Gun2Proportion, Gun2Integral, Gun2Derivatite, Gun2Alpha, Gun2DeltaT, Gun2SumAboveLimit, Gun2SumBelowLimit);
	// End MOTOR GUN INIT

	// Start MOTOR RULO GET BALL INIT

	// End MOTOR RULO GET BALL INIT
}

void encoderGun_ResetCount(Encoder_t *enc, int *count_X1)
{
	count_X1 = 0;
	enc->vel_Real = 0;
	enc->vel_Pre = 0;
}

void gun_ResetEncoder(int *gunCount1, int *gunCount2)
{
	encoderGun_ResetCount(&ENC_Gun1, gunCount1);
	encoderGun_ResetCount(&ENC_Gun2, gunCount2);
}

void gun_VelCal(int gunCount1, int gunCount2) {
	VelCal(&ENC_Gun1, gunCount1, DCEncoderPerRound, Gun1DeltaT);
	VelCal(&ENC_Gun2, gunCount2, DCEncoderPerRound, Gun2DeltaT);
}

void gun_PIDSpeed1(float Target1) {
	PID_Calculate(&PID_Gun1, Target1, ENC_Gun1.vel_Real);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PID_Gun1.uHat);
}

void gun_PIDSpeed2(float Target2) {
	PID_Calculate(&PID_Gun2, Target2, ENC_Gun2.vel_Real);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PID_Gun2.uHat);
}

void VelCal(Encoder_t *enc, int count_X1, uint32_t count_PerRevol, float deltaT)
{
	enc->vel_Real = ((count_X1 - enc->count_Pre) / deltaT) / (count_PerRevol) * 60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real + 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_Pre = count_X1;
}

void RB1_CollectBallMotor_Init()
{
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

static void CollectBallMotorSpeedUp()
{
	if (HAL_GetTick() - collectBallTickTime > 150 && collectBallPWM <= 1000) {
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

