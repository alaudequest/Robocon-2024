/*
 * RTC_Format.h
 *
 *  Created on: May 12, 2023
 *      Author: SpiritBoi
 */

#ifndef INC_RTC_FORMAT_H_
#define INC_RTC_FORMAT_H_

#include "main.h"
#include "string.h"
#include "stdio.h"

typedef struct RTC_t
{
  int8_t year;
  int8_t month;
  int8_t day;
  int8_t weekday;
  int8_t hour;
  int8_t minute;
  int8_t second;
}RTC_t;

RTC_t RTC_GetTimeFromString(char *s);
void RTC_PackTimeToString(RTC_t t, char *s);
uint8_t RTC_ConvertTickElapsedToMinute(uint16_t input_RTC_TickCount);
uint8_t RTC_ConvertTickElapsedToSecond(uint16_t input_RTC_TickCount);
uint8_t RTC_ConvertTickElapsedToHour(uint16_t input_RTC_TickCount);
#endif /* INC_RTC_FORMAT_H_ */
