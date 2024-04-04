#ifndef INC_INVERSEKINE_H_
#define INC_INVERSEKINE_H_

//#include "math.h"
#include "SwerveModule.h"
#include "math.h"
#include "main.h"
typedef enum InverseKinematicProcedure{
	INV_PROC_BEGIN = 1,
	CALC_WHEEL_VECTOR,
	CALC_WHEEL_OPT,
	CALC_WHEEL_VELOC,
	INV_PROC_END,
}InverseKinematicProcedure;

typedef void (*ptnCpltCallback)(ModuleID, float, float);

void invkine_CalWheelVector(ModuleID ID, float u, float v, float r);
InverseKinematicProcedure invkine_GetStep();
void invkine_Begin();
HAL_StatusTypeDef  invkine_Implementation(ModuleID ID, float u, float v, float r,void (*ptnCpltCallback)(ModuleID,float, float));

InverseKinematicProcedure invkine_GetInvCalStep();
void invkine_SetInvCalStep(InverseKinematicProcedure Step);
#endif
