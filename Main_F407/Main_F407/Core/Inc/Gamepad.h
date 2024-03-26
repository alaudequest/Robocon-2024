/*
 * Gamepad.h
 *
 *  Created on: Oct 24, 2023
 *      Author: Admin
 */

#ifndef INC_GAMEPAD_H_
#define INC_GAMEPAD_H_

#include "main.h"
#include "math.h"

typedef struct _GamePad{
	uint8_t Status;

	uint8_t XLeft;
	uint8_t YLeft;

	uint8_t XRight;
	uint8_t YRight;

	uint8_t Left;
	uint8_t Up;
	uint8_t Right;
	uint8_t Down;

	uint8_t Square;
	uint8_t Triangle;
	uint8_t Circle;
	uint8_t Cross;

	uint8_t L1;
	uint8_t L2;
	uint8_t L3;

	uint8_t R1;
	uint8_t R2;
	uint8_t R3;

	uint8_t Touch;

	uint8_t Charge;
	uint8_t Battery;

	float XLeftCtr;
	float YLeftCtr;
	float XRightCtr;
} _GamePad;


void GamepPadHandle(_GamePad *pad,uint8_t *DataTayGame);


#endif /* INC_GAMEPAD_H_ */
