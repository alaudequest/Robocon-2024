/*
 * BoardParamter.c
 *
 *  Created on: Apr 28, 2024
 *      Author: ADMIN
 */

#include "BoardParameter.h"

CustomGamepad_t _customGamepad;
TeamColor teamColor = 0;

void BrdParam_SetCustomGamepad(CustomGamepad_t gamepad)
{
	_customGamepad = gamepad;
}

CustomGamepad_t BrdParam_GetCustomGamepad()
{
	return _customGamepad;
}

void BrdParam_SetTeamColor(TeamColor color)
{
	teamColor = color;
}

TeamColor BrdParam_GetTeamColor()
{
	return teamColor;
}
