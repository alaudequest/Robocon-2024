/*
 * ActuatorGun.c
 *
 *  Created on: Mar 24, 2024
 *      Author: namdhay
 */

#include "ActuatorGun.h"
#include "PID.h"
#include "Encoder.h"
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim8;
static Encoder_t encGun1;
static Encoder_t encGun2;
static PID_Param PID_Gun1;
static PID_Param PID_Gun2;

static uint8_t pidCurrentTickTimeGun1_ms = 0;
static uint8_t pidCurrentTickTimeGun2_ms = 0;
static Acceleration_t accelGun1, accelGun2;
float gun1TargetSpeed, gun2TargetSpeed;
static void RB1_Gun_AccelerateInit();

void RB1_Gun_Init() {
	// Start MOTOR GUN INIT

	// Bắn 2
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	// Bắn 1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	PID_SetParameters(&PID_Gun1, Gun1Proportion, Gun1Integral, Gun1Derivatite, Gun1Alpha);
	PID_SetSaturate(&PID_Gun1, Gun1SumAboveLimit, Gun1SumBelowLimit);
	encGun1.deltaT = PID_Gun1.deltaT = Gun1DeltaT;

	PID_SetParameters(&PID_Gun2, Gun2Proportion, Gun2Integral, Gun2Derivatite, Gun2Alpha);
	PID_SetSaturate(&PID_Gun2, Gun2SumAboveLimit, Gun2SumBelowLimit);
	encGun2.deltaT = PID_Gun2.deltaT = Gun2DeltaT;
	RB1_Gun_AccelerateInit();
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
	if (accelGun1.lockNumStep == true) {
		accelGun1.accelTick_ms++;
	}
	if (accelGun2.lockNumStep == true) {
		accelGun2.accelTick_ms++;
	}
}

static float absf(float num)
{
	if (num >= 0)
		return num;
	else
		return num * -1;
}

static void CalculateAccelValue(Acceleration_t *a)
{
	// Lấy số step ban đầu và khóa giá trị step đó lại
	if (a->lockNumStep == false && (a->targetValue != a->currentOutputValue)) {
		a->valueStep = absf(a->targetValue - a->currentOutputValue) / a->numStep;
		a->lockNumStep = true;
	}
	// Nếu tick hiện tại chưa đạt tới thời điểm thay đổi giá trị gia tốc thì return
	if (a->accelTick_ms < a->accelTimeStep_ms)
		return;
	// Nếu đang gia tốc và giá trị hiện tại nhỏ hơn giá trị đặt trước
	if (a->targetValue > a->currentOutputValue)
		a->currentOutputValue += a->valueStep;
	// Nếu đang giảm tốc và giá trị đặt nhỏ hơn giá trị hiện tại
	else if (a->targetValue < a->currentOutputValue)
		a->currentOutputValue -= a->valueStep;
	// Nếu giá trị hiện tại xấp xỉ giá trị đặt thì dừng quá trình gia tốc
	if (absf(a->targetValue - a->currentOutputValue) < a->valueStep) {
		a->currentOutputValue = a->targetValue;
		a->lockNumStep = false;
	}
	a->accelTick_ms = 0;
}

void RB1_CalculateRuloGunPIDSpeed()
{
	RB1_VelocityCalculateOfGun();
	float uHat = PID_Calculate(&PID_Gun1, gun1TargetSpeed, encGun1.vel_Real);
//	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, uHat);
	uHat = PID_Calculate(&PID_Gun2, gun2TargetSpeed, encGun2.vel_Real);
//	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, uHat);

	if (pidCurrentTickTimeGun1_ms >= Gun1DeltaT * 1000) { // DeltaT = 0.01s = 10ms
		pidCurrentTickTimeGun1_ms = 0;
	}
	if (pidCurrentTickTimeGun2_ms >= Gun2DeltaT * 1000) {
		pidCurrentTickTimeGun2_ms = 0;
	}
}

void RB1_Gun_AccelerateInit() {
	accelGun1.accelTimeStep_ms = 150;
	accelGun1.numStep = 7;

	accelGun2.accelTimeStep_ms = 150;
	accelGun2.numStep = 7;
}

void RB1_Gun_Start(float gun1TargetSpeed, float gun2TargetSpeed)
{
	accelGun1.targetValue = gun1TargetSpeed;
	accelGun2.targetValue = gun2TargetSpeed;
	accelGun1.accelTick_ms = 0;
	accelGun2.accelTick_ms = 0;
}

void RB1_Gun_Stop() {
	accelGun1.accelTick_ms = 0;
	accelGun2.accelTick_ms = 0;
	accelGun1.targetValue = 0;
	accelGun2.targetValue = 0;
}

void RB1_SetTargetSpeedGun1(float targetSpeed)
{
//	accelGun1.targetValue = targetSpeed;
	gun1TargetSpeed = targetSpeed;
}

void RB1_SetTargetSpeedGun2(float targetSpeed)
{
//	accelGun2.targetValue = targetSpeed;
	gun2TargetSpeed = targetSpeed;
}



void RB1_CollectBallMotor_Init()
{

}

void RB1_CollectBallMotor_ControlSpeed()
{

}

void RB1_CollectBallMotor_On()
{
	HAL_GPIO_WritePin(RelayCollectBallLeft_GPIO_Port, RelayCollectBallLeft_Pin, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(RelayCollectBallRight_GPIO_Port, RelayCollectBallRight_Pin, 1);
}

void RB1_CollectBallMotor_Off()
{
	HAL_GPIO_WritePin(RelayCollectBallLeft_GPIO_Port, RelayCollectBallLeft_Pin, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(RelayCollectBallRight_GPIO_Port, RelayCollectBallRight_Pin, 0);
}

void RB1_GunIncreaseTickTimerInInterrupt()
{
	pidCurrentTickTimeGun1_ms++;
	pidCurrentTickTimeGun2_ms++;
}

float RB1_GetSpeedRuloShootBall1()
{
	return encGun1.vel_Fil;
}

float RB1_GetSpeedRuloShootBall2()
{
	return encGun2.vel_Fil;
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
