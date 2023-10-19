/*
 * StringUtility.h
 *
 *  Created on: Apr 8, 2023
 *      Author: SpiritBoi
 */

#ifndef STRING_UTILITY_H_
#define STRING_UTILITY_H_
#include "main.h"
#ifdef CONFIG_USE_STRING_UTILITY
#include "string.h"
#include "stdlib.h"

#define NUM_STRING 5
#define STRING_LENGTH 15

typedef enum{
	STRING_OK,
	STRING_INVALID_ARG,
	STRING_NULL,
	STRING_BUFFER_OVERFLOW,
	STRING_LENGTH_TOO_LONG,
	STRING_KEY_FOUND,
	STRING_KEY_NOT_FOUND,
}StringStatus_t;

StringStatus_t StrUtil_TokenMessage(char *String,char* delimiter);
StringStatus_t StrUtil_TokenKeyValue(char *String);
StringStatus_t StrUtil_SearchKey(char *String, char* KeySearch);
void StrUtil_ReturnValueToString(char *s,StringStatus_t retVal);
double StrUtil_ConvertToDouble(char *String);
int64_t StrUtil_ConvertToInt64(char *String);

#endif /*CONFIG_USE_STRING_HANDLE*/
#endif /*STRINGHANDLE_H_*/
