/*
 * Flag.h
 *
 *  Created on: Mar 29, 2023
 *      Author: SpiritBoi
 */

#ifndef _FLAG_H_
#define _FLAG_H_
#include "main.h"
#ifdef CONFIG_USE_FLAG

#define CHECKFLAG(FlagGroup,FlagBit) ((((FlagGroup) & (1UL << FlagBit)) == (1UL << FlagBit)) ? 1 : 0)
#define SETFLAG(FlagGroup,FlagBit) ((FlagGroup) |= (1UL << FlagBit))
#define CLEARFLAG(FlagGroup,FlagBit) (FlagGroup &= ~(1UL << FlagBit))



#endif /*CONFIG_USE_FLAG*/
#endif /*_FLAG_H_*/
