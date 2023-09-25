/*
 * BoardParameter.h
 *
 *  Created on: Sep 23, 2023
 *      Author: KHOA
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_
#include "main.h"
#include "PID.h"

/*-----------------------------Begin:BLDC Macro-------------------------------*/
#define _BLDCEncoderPerRound 				  200
#define _BLDCGearRatio 						  2.5
#define _BLDCDeltaT							0.001
/*-----------------------------End:BLDC Macro---------------------------------*/

/*-----------------------------Begin:DC Macro---------------------------------*/
#define DCDeltaT 							0.001
#define DCEncoderPerRound 					 1000
#define DCGearRatio 						3.535
/*-----------------------------End:DC Macro-----------------------------------*/


typedef struct BoardParameter {
	float bldcSpeed;
	float dcAngle;
	uint16_t encBLDC;
	uint16_t encDC;
	PID_Param pid_DC_Angle;
	PID_Param pid_DC_Speed;
	PID_Param pid_BLDC_Speed;
}BoardParameter ;

#endif /* INC_BOARDPARAMETER_H_ */
