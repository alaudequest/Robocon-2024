/*
 * PID.c
 *
 *  Created on: Jul 30, 2023
 *      Author: LENOVO
 */

#include "PID.h"
#include "stdlib.h"

 #ifdef PID_EN
//-----------------------------------------------Begin: Setting Parameter for PID------------------------------------------//
void Pid_SetParam(PID_Param *pid,double kP,double kI,double kD,double alpha,double deltaT,double uI_AboveLimit,double uI_BelowLimit,double u_AboveLimit,double u_BelowLimit)
{
//----------------------Term-----------------------//
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->alpha = alpha;
//----------------------Sample Time----------------//
	pid->deltaT = deltaT;
//----------------------Limit----------------------//
	pid->uI_AboveLimit = uI_AboveLimit;
	pid->uI_BelowLimit = uI_BelowLimit;
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}

//-----------------------------------------------End: Setting Parameter for PID--------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------Begin: Change Parameter for PID-------------------------------------------//

void Pid_uI_PreSetParam(PID_Param *pid,double uI_AboveLimit,double uI_BelowLimit)
{
	pid->uI_AboveLimit = uI_AboveLimit;
	pid->uI_BelowLimit = uI_BelowLimit;
}



void Pid_u_PresetParam(PID_Param *pid,double u_AboveLimit,double u_BelowLimit)
{
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}



void Pid_Term_PresetParam(PID_Param *pid,double kP,double kI,double kD)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}

//-----------------------------------------------End: Change Parameter for PID---------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------Begin: Calculating PID---------------------------------------------------//

void Pid_Cal(PID_Param *pid,double Target,double CurrVal)
{
//-----------------------Input-------------------------//
	pid->Target = Target;
	pid->CurrVal = CurrVal;
	pid->e = pid->Target - pid->CurrVal;

//-----------------------Propotion Term----------------//
	pid->uP = pid->kP*pid->e;

//-----------------------Integral Term-----------------//
	pid->uI = pid->uI_Pre + pid->kI*pid->e*pid->deltaT;
	pid->uI = pid->uI > pid->uI_AboveLimit ? pid->uI_AboveLimit : pid->uI;
	pid->uI = pid->uI < pid->uI_BelowLimit ? pid->uI_BelowLimit : pid->uI;

//-----------------------Derivative Term---------------//
	pid->uD = pid->kD*(pid->e - pid->e_Pre)/pid->deltaT;
	pid->uD_Fil = (1-pid->alpha)*pid->uD_FilPre+pid->alpha*pid->uD;

//-----------------------Previous Value----------------//
	pid->e_Pre = pid->e;
	pid->uI_Pre = pid->uI;
	pid->uD_FilPre = pid->uD_Fil;

//-----------------------Sum---------------------------//
	pid->u = pid->uP + pid->uI + pid->uD;
	pid->u = pid->u > pid->u_AboveLimit ? pid->u_AboveLimit : pid->u;
	pid->u = pid->u < pid->u_BelowLimit ? pid->u_BelowLimit : pid->u;

//	return pid->u;
}

//-----------------------------------------------End: Calculating PID-----------------------------------------------------//
#endif


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
//	motor->Mode = Mode;
	if(Input < 0)
	{
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,1000-motor->Pwm);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,1000);
	}
	else if(Input > 0)
	{
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,1000);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,1000-motor->Pwm);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel1,1000);
		__HAL_TIM_SET_COMPARE(motor->htim1,motor->Channel2,1000);
	}
}


#endif
