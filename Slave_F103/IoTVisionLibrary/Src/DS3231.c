/*
 * DS3231.cpp
 *
 *  Created on: Nov 29, 2022
 *      Author: anh
 */
#include "DS3231.h"
#ifdef CONFIG_USE_DS3231

static I2C_HandleTypeDef *_i2c;
static uint8_t decToBcd(int val){
  return (uint8_t)((val/10*16) + (val%10));
}

static int bcdToDec(uint8_t val){
  return (int)((val/16*10) + (val%16));
}

static void DS3231_Temp_Conv(void){
	uint8_t status = 0;
	uint8_t control = 0;

	HAL_I2C_Mem_Read(_i2c, DS3231_ADDRESS, 0x0F, 1, &status, 1, 1000);

	if (!(status & 0x04)){
		HAL_I2C_Mem_Read(_i2c, DS3231_ADDRESS, 0x0E, 1, &control, 1, 1000);
		HAL_I2C_Mem_Write(_i2c, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)(control|(0x20)), 1, 1000);
	}
}

void DS3231_Init(I2C_HandleTypeDef *i2c){
	_i2c = i2c;

	DS3231_Temp_Conv();
}

void DS3231_SetTime(DS3231_Time_t time){
	uint8_t set_time[7];
	set_time[0] = decToBcd(time.seconds );
	set_time[1] = decToBcd(time.minutes);
	set_time[2] = decToBcd(time.hour);
	set_time[3] = decToBcd(time.dayofweek);
	set_time[4] = decToBcd(time.dayofmonth);
	set_time[5] = decToBcd(time.month);
	set_time[6] = decToBcd(time.year);

	while(HAL_I2C_Mem_Write(_i2c, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000)!=HAL_OK);
}

void DS3231_GetTime(DS3231_Time_t *time){
	uint8_t get_time[7];

	HAL_I2C_Mem_Read(_i2c, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);

	time->seconds 	 = bcdToDec(get_time[0]);
	time->minutes 	 = bcdToDec(get_time[1]);
	time->hour 		 = bcdToDec(get_time[2]);
	time->dayofweek  = bcdToDec(get_time[3]);
	time->dayofmonth = bcdToDec(get_time[4]);
	time->month 	 = bcdToDec(get_time[5]);
	time->year       = bcdToDec(get_time[6]);
}

float DS3231_GetTemp(void){
	uint8_t temp[2];

	HAL_I2C_Mem_Read(_i2c, DS3231_ADDRESS, 0x11, 1, temp, 2, 1000);
	return (float)(temp[0] + (temp[1]>>6)/4.0);
}
#endif


