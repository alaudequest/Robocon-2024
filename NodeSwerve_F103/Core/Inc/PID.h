#ifndef _PID_H_
#define _PID_H_
#include "main.h"
#ifdef CONFIG_USE_PID
#include "math.h"
#include <stdlib.h>
#include <stdint.h>


typedef struct PID_Param{
	//---------Input Parameters-----------//
    float e;
    float e_Pre;
    float deltaT;
	//---------Propotion Parameters-------//
    float kP;
    float uP;
	//---------Intergral Parameters-------//
    float kI;
    float uI;
    float kB;// gain value to control anti-windup of Intergral
	//---------Derivative Parameters------//
    float kD;
    float uD;
    float uD_Fil;
    float uD_FilPre;
    float alpha;
	//---------Sum Parameters-------------//
    float u;
    float uHat; // the result of 'u' value after passing "Saturation" stage
    float u_AboveLimit;
    float u_BelowLimit;

}PID_Param;

void PID_Init(PID_Param *pid);
float PID_Cal(PID_Param *pid,float Target_set,float CurrVal_set);
float PID_Cal_BLDC(PID_Param *pid,float Target_set,float CurrVal_set);
#endif
#endif
