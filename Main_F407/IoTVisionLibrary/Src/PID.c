#include "PID.h"
#ifdef CONFIG_USE_PID

#define e               pid->e
#define e_Pre           pid->e_Pre
#define deltaT          pid->deltaT
#define kP              pid->kP
#define uP              pid->uP
#define kI              pid->kI
#define uI              pid->uI
#define kB				pid->kB
#define kD              pid->kD
#define uD              pid->uD
#define uD_Fil          pid->uD_Fil
#define uD_FilPre 		pid->uD_FilPre
#define alpha   		pid->alpha
#define u				pid->u
#define uHat			pid->uHat
#define u_AboveLimit 	pid->u_AboveLimit
#define u_BelowLimit 	pid->u_BelowLimit

/**
 * @brief Tính toán giá trị điện áp xuất ra bằng công thức PID
 * @param *pid con trỏ tới struct PID_Param
 * @param Target_set giá trị xác lập mong muốn
 * @param CurrVal_set giá trị feedback của hệ thống
 * @return
 */

float PID_Cal(PID_Param *pid,float Target_set,float CurrVal_set)
{
//-----------------------Input-------------------------//
	e = Target_set - CurrVal_set;

//	if((e>-1.5)&&(e<1.5))e = 0;
	if(!kI) kB = 0;
	else kB = 1/deltaT;

//-----------------------Propotion Term----------------//
	uP = kP*e;

//-----------------------Integral Term-----------------//
	uI +=(kI*e + kB*(-u + uHat))*deltaT;

//-----------------------Derivative Term---------------//
	uD = kD*(e - e_Pre)/deltaT;
	uD_Fil = (1-alpha)*uD_FilPre+alpha*uD;

//-----------------------Previous Value----------------//
	e_Pre = e;
	uD_FilPre = uD_Fil;

//-----------------------Sum---------------------------//
	u = uP + uI + uD;
	if(u >= u_AboveLimit) uHat = u_AboveLimit;
	else if(u <= u_BelowLimit) uHat = u_BelowLimit;
	else uHat = u;
	return uHat;
}

#endif
