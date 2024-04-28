/*
 * BoardParameter.h
 *
 *  Created on: Apr 28, 2024
 *      Author: ADMIN
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_
#include "main.h"

typedef struct CustomGamepad_t {
	uint8_t ballMatrix[6];
	uint8_t siloNum;
} CustomGamepad_t;
typedef uint8_t PaddyRice_t;

typedef struct HarvestZoneBallPosition_t {
	uint8_t row;
	uint8_t column;
} HarvestZoneBallPosition_t;

typedef enum TeamColor {
	RED_TEAM = 1,
	BLUE_TEAM = 2,
} TeamColor;

void BrdParam_SetCustomGamepad(CustomGamepad_t gamepad);
CustomGamepad_t BrdParam_GetCustomGamepad();
void BrdParam_SetTeamColor(TeamColor color);
TeamColor BrdParam_GetTeamColor();
void BrdParam_SetBallSuccess(uint8_t ballGetSuccess);
uint8_t BrdParam_GetBallSuccess();
#endif /* INC_BOARDPARAMETER_H_ */
