/*
 * BoardParamter.c
 *
 *  Created on: Apr 28, 2024
 *      Author: ADMIN
 */

#include "BoardParameter.h"

uint8_t _ballGetSuccess;
CustomGamepad_t _customGamepad;
TeamColor teamColor = 0;

void BrdParam_SetTeamColor(TeamColor color)
{
	teamColor = color;
}

TeamColor BrdParam_GetTeamColor()
{
	return teamColor;
}

#if defined(BOARD_MAINF4_ROBOT1)

#elif defined(BOARD_MAINF4_ROBOT2)

void BrdParam_SetCustomGamepad(CustomGamepad_t gamepad)
{
	_customGamepad = gamepad;
}

CustomGamepad_t BrdParam_GetCustomGamepad()
{
	return _customGamepad;
}

void BrdParam_SetBallSuccess(uint8_t ballGetSuccess)
{
	_ballGetSuccess = ballGetSuccess;
}

uint8_t BrdParam_GetBallSuccess()
{
	return _ballGetSuccess;
}

#else
#error "Not define board selected in main.h"
#endif
