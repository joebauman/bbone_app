/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMOTIMER_H_
#define _DEMOTIMER_H_

/******************************************************************************
**                     EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern volatile unsigned int tmrFlag;
extern unsigned int tmrClick;
extern volatile unsigned int tmr4Flag;

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void Timer2IntRegister(void);
extern void Timer2Config(void);
extern void Timer2IntEnable(void);
extern void Timer2Start(void);
extern void Timer2Stop(void);

extern void Timer4IntRegister(void);
extern void Timer4Config(void);
extern void Timer4IntEnable(void);
extern void Timer4Start(void);
extern void Timer4Stop(void);

#endif
