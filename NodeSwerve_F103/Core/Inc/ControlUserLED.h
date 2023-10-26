/*
 * ControlUserLED.h
 *
 *  Created on: Sep 27, 2023
 *      Author: SpiritBoi
 */

#ifndef INC_CONTROLUSERLED_H_
#define INC_CONTROLUSERLED_H_

#include "main.h"

typedef enum LED_Mode{
	LED_BLINK,
	LED_STATE,
	LED_FADE,
}LED_Mode;

typedef struct UserLED_t{
	uint16_t delay;
	uint8_t mode;
}UserLED_t;
void runBlueLED();
#endif /* INC_CONTROLUSERLED_H_ */
