/*
 * Robot1_InShootBallTime.h
 *
 *  Created on: Apr 17, 2024
 *      Author: KHOA
 */

#ifndef INC_ROBOT1_INSHOOTBALLTIME_H_
#define INC_ROBOT1_INSHOOTBALLTIME_H_

#include "main.h"
#include "Gamepad.h"
#include "RB1ActuatorValve.h"

void ShootBallTime_Stop();
void ShootBallTime_Start(_GamePad *gamepad);
void ShootBallTime_Handle();
bool IsInShootBallTime();

#endif /* INC_ROBOT1_INSHOOTBALLTIME_H_ */
