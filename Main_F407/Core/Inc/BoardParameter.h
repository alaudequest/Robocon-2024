/*
 * BoardParameter.h
 *
 *  Created on: Sep 23, 2023
 *      Author: SpiritBoi
 */

#ifndef INC_BOARDPARAMETER_H_
#define INC_BOARDPARAMETER_H_
#include "main.h"
#include "PID.h"
#include "stdbool.h"
#include "InverseKinematic.h"

#define FLASH_ROBOT_PLAYGROUND_DATA_ADDRESS 0x080e0000


typedef enum SignalButtonColor {
	SIGBTN_RED = 1,
	SIGBTN_YELLOW,
	SIGBTN_BLUE,
	SIGBTN_GREEN,
} SignalButtonColor;
typedef void (*pSignalBtnPressed)(SignalButtonColor);

typedef enum TeamColor {
	RED_TEAM = 1,
	BLUE_TEAM = 2,
} TeamColor;

#if defined(BOARD_MAINF4_ROBOT1)
typedef uint8_t PaddyRice_t;
typedef enum MainF4Robot1TypePID {
	PID_RULO_1 = 1,
	PID_RULO_2,
	PID_ROBOT_THETA,
} MainF4Robot1TypePID;
#elif defined(BOARD_MAINF4_ROBOT2)

typedef struct CustomGamepad_t {
	uint8_t ballMatrix[6];
	uint8_t siloNum;
} CustomGamepad_t;

typedef struct HarvestZoneBallPosition_t {
	uint8_t row;
	uint8_t column;
} HarvestZoneBallPosition_t;
#endif

typedef struct Robot2PlayGroundData{
	CustomGamepad_t gp;
	uint8_t ballCollectSuccess;
	uint8_t ballInSilo[5];
	HarvestZoneBallPosition_t nextBall;
	TeamColor teamColor;
}Robot2PlayGroundData;



void BrdParam_SetTeamColor(TeamColor color);
TeamColor BrdParam_GetTeamColor();

#if defined(BOARD_MAINF4_ROBOT1)

#elif defined(BOARD_MAINF4_ROBOT2)
void BrdParam_SetCustomGamepad(CustomGamepad_t gamepad);
CustomGamepad_t BrdParam_GetCustomGamepad();
void BrdParam_SetBallSuccess(uint8_t ballGetSuccess);
uint8_t BrdParam_GetBallSuccess();
#endif

void BrdParam_SetIsOnLoadDataFromFlash(bool isOnLoadDataFromFlash);
void BrdParam_SaveDataToFlash();

void ProcessDelay(uint32_t delayMs, uint8_t *step);
void RobotSignalButton_RegisterButtonPressedCallback(void (*pSignalBtnPressed)(SignalButtonColor));
HAL_StatusTypeDef BuzzerBeep_Start(uint8_t repeatTime, uint32_t onDelayMs, uint32_t offDelayMs);
void BuzzerBeepProcess();
void RobotSignalButton_ScanButton();
#endif /* INC_BOARDPARAMETER_H_ */
