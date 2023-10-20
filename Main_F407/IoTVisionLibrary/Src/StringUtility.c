/*
 * StringUtility.c
 *
 *  Created on: Apr 8, 2023
 *      Author: SpiritBoi
 */

#include "StringUtility.h"
#ifdef CONFIG_USE_STRING_UTILITY

char ArrBuffStr[NUM_STRING][STRING_LENGTH];

StringStatus_t StrUtil_TokenMessage(char **TokenBuffer,char *String,char* delimiter)
{
	if(!String || !TokenBuffer) return STRING_NULL;
	char *p;
	uint8_t i=1;
	p = strtok(String,delimiter);
	if(strlen(p) < STRING_LENGTH){
		strcpy(ArrBuffStr[0],p);
	} else return STRING_LENGTH_TOO_LONG;
	while(p!=NULL){
		p = strtok(NULL,delimiter);
		if(!p) break;
		if(strlen(p) < STRING_LENGTH) strcpy(ArrBuffStr[i],p);
		else return STRING_LENGTH_TOO_LONG;
		i++;
		if(i==NUM_STRING && p!=NULL) return STRING_BUFFER_OVERFLOW;
		else if(i==NUM_STRING) break;
	}
	return STRING_OK;
}

StringStatus_t StrUtil_SearchKey(char *String, char* KeySearch)
{
	char *c;
	for(uint8_t i=0;i<sizeof(NUM_STRING);i++)
	{
		c=strstr(ArrBuffStr[i],KeySearch);
		if(c!=NULL) return STRING_KEY_FOUND;
	}
	return STRING_KEY_NOT_FOUND;
}

void StrUtil_ReturnValueToString(char *s,StringStatus_t retVal)
{
    switch(retVal){
        case STRING_KEY_FOUND: strcpy(s,"STRING_KEY_FOUND"); break;
        case STRING_KEY_NOT_FOUND: strcpy(s,"STRING_KEY_NOT_FOUND"); break;
        case STRING_OK: strcpy(s,"STRING_OK"); break;
        case STRING_BUFFER_OVERFLOW: strcpy(s,"STRING_BUFFER_OVERFLOW"); break;
        case STRING_LENGTH_TOO_LONG: strcpy(s,"STRING_LENGTH_TOO_LONG"); break;
        case STRING_INVALID_ARG: strcpy(s,"STRING_INVALID_ARG"); break;
        case STRING_NULL: strcpy(s,"STRING_NULL"); break;
    }
}

double StrUtil_ConvertToDouble(char *String)
{
    return strtod(String,NULL);
}

int64_t StrUtil_ConvertToInt64(char *String)
{
    return strtol(String,NULL,10);
}
#endif

