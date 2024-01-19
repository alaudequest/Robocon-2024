/*
 * BoardParameter.c
 *
 *  Created on: Jan 18, 2024
 *      Author: NamDHay
 */

#include "BoardParameter.h"

BoardParameter_t brdParam;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void brd_Init()
{
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	Motor_Init(&brdParam.rotary, MOTOR_PWM, 0, 0, &htim1, TIM_CHANNEL_2, TIM_CHANNEL_3);
	Motor_Init(&brdParam.gun1, MOTOR_PWM, 0, 0, &htim2, TIM_CHANNEL_3, 0);
	Motor_Init(&brdParam.gun2, MOTOR_PWM, 0, 0, &htim2, TIM_CHANNEL_4, 0);
	Motor_Init(&brdParam.ball1, MOTOR_LL, RuloBall1_GPIO_Port, RuloBall1_Pin, NULL, 0, 0);
	Motor_Init(&brdParam.ball2, MOTOR_LL, RuloBall2_GPIO_Port, RuloBall2_Pin, NULL, 0, 0);

	encoder_Init(&brdParam.encGun2, &htim3, 200,0.005, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7);

	brdParam.pidAngle.kP = 0.1;
	brdParam.pidAngle.kI = 5;
	brdParam.pidAngle.kD = 0;
	brdParam.pidAngle.alpha = 0;
	brdParam.pidAngle.deltaT = PIDDeltaT;
	brdParam.pidAngle.u_AboveLimit = 1000;
	brdParam.pidAngle.u_BelowLimit = -1000;
	brdParam.pidAngle.kB = 1/PIDDeltaT;

	brdParam.pidGun2.kP = 5;
	brdParam.pidGun2.kI = 0;
	brdParam.pidGun2.kD = 0.04;
	brdParam.pidGun2.alpha = 0.8;
	brdParam.pidGun2.deltaT = PIDDeltaT;
	brdParam.pidGun2.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidGun2.u_BelowLimit = DC_SUM_BELOW_LIMIT;
	brdParam.pidGun2.kB = 1/PIDDeltaT;

	brdParam.pidGun1.kP = 1;
	brdParam.pidGun1.kI = 200;
	brdParam.pidGun1.kD = 0;
	brdParam.pidGun1.alpha = 0;
	brdParam.pidGun1.deltaT = PIDDeltaT;
	brdParam.pidGun1.u_AboveLimit = DC_SUM_ABOVE_LIMIT;
	brdParam.pidGun1.u_BelowLimit = DC_SUM_BELOW_LIMIT;
	brdParam.pidGun1.kB = 1/PIDDeltaT;
}
