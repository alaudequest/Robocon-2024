/*
 * DriveMotor.c
 *
 *  Created on: Sep 5, 2023
 *      Author: defaultuser0
 */
#include "DriveMotor.h"
#include "stdlib.h"

#ifdef ENC_EN
void EncoderSetting(EncoderRead *enc,TIM_HandleTypeDef *htim,int count_PerRevol,double deltaT)
{
	enc->htim = htim;
	enc->count_PerRevol = count_PerRevol;
	enc->deltaT = deltaT;
}
void SpeedReadOnly(EncoderRead *enc)
{
	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = (enc->count_X4/enc->deltaT)/(enc->count_PerRevol)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_X4 = 0;
}



void SpeedReadNonReset(EncoderRead *enc){

	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = ((enc->count_X4-enc->count_Pre)/enc->deltaT)/(enc->count_PerRevol*4)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_Pre = enc->count_X4;
}

double CountRead(EncoderRead *enc,uint8_t count_mode){
	enc->count_Mode = count_mode;
	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);

	if (enc->count_Mode == count_ModeX4)
	{
		return enc->count_X4;
	}else if (enc->count_Mode == count_ModeX1)
	{
		enc->count_X1 = enc->count_X4/4;
		return enc->count_X1;
	}else if (enc->count_Mode == count_ModeDegree)
	{
		enc->Degree = enc->count_X4*360/(enc->count_PerRevol*4);
		return enc->Degree;
	}else {
		return 0;
	}
}

void ResetCount(EncoderRead *enc,uint8_t command)
{
	if (command == 1)
	{
		__HAL_TIM_SET_COUNTER(enc->htim,0);
		enc->count_X4 = 0;
	}
}
#endif



#ifdef MOTOR_EN

void BLDC_Drive_RedBoard(MotorDrive *motor,TIM_HandleTypeDef *htim1,int Input,unsigned int Channel1)
{
	motor->htim1 = htim1;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;
	if (Input>0)
	{
		HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, 1);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
	}
	else
	{
		HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, 0);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
	}
}

void DC_Drive_BTS(MotorDrive *motor,TIM_HandleTypeDef *htim1,uint16_t Mode,int Input,unsigned int Channel1,unsigned int Channel2)
{
	motor->htim1 = htim1;
	motor->Mode = Mode;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;
	motor->Channel2 = Channel2;

	if(motor->Mode==0){
		if(Input < 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Pwm);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,0);
		}
		else if(Input > 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,0);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Pwm);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,0);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,0);
		}
	}else{
		if(Input < 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Mode-motor->Pwm);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Mode);
		}
		else if(Input > 0)
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Mode);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Mode-motor->Pwm);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,motor->Mode);
			__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,motor->Mode);
		}
	}
}


#endif

