/*
 * Gamepad.c
 *
 *  Created on: Oct 24, 2023
 *      Author: Admin
 */
#include"Gamepad.h"

void GamepPadHandle(_GamePad *pad,uint8_t *DataTayGame){
	pad->Status = 1;

	pad->XLeft = DataTayGame[1];
	pad->YLeft = DataTayGame[2];

	pad->XRight = DataTayGame[3];
	pad->YRight = DataTayGame[4];

	pad->Left = (DataTayGame[5] >> 7) & 1;
	pad->Up = (DataTayGame[5] >> 6) & 1;
	pad->Right = (DataTayGame[5] >> 5) & 1;
	pad->Down = (DataTayGame[5] >> 4) & 1;

	pad->Square = (DataTayGame[5] >> 3) & 1;
	pad->Triangle = (DataTayGame[5] >> 2) & 1;
	pad->Circle = (DataTayGame[5] >> 1) & 1;
	pad->Cross = DataTayGame[5] & 1;

	pad->L1 = (DataTayGame[6] >> 7) & 1;
	pad->L2 = (DataTayGame[6] >> 6) & 1;
	pad->R1 = (DataTayGame[6] >> 5) & 1;
	pad->R2 = (DataTayGame[6] >> 4) & 1;

	pad->Touch = (DataTayGame[6] >> 3) & 1;
	pad->Charge = (DataTayGame[6] >> 2) & 1;

	pad->L3 = (DataTayGame[6] >> 1) & 1;
	pad->R3 = DataTayGame[6] & 1;

	pad->Battery = DataTayGame[7];

	pad->XLeftCtr = ((pad->XLeft-125))*110/125;
	pad->YLeftCtr = ((pad->YLeft-125))*110/125;
	pad->XRightCtr =(((pad->XRight-125))*300/125);

	if ((pad->XLeftCtr > -20)&&(pad->XLeftCtr < 20))pad->XLeftCtr = 0;
	if ((pad->YLeftCtr > -20)&&(pad->YLeftCtr < 20))pad->YLeftCtr = 0;
	if ((pad->XRightCtr > -20)&&(pad->XRightCtr < 20))pad->XRightCtr = 0;
}
