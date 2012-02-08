/*
 * \file   demoSdRw.h
 *
 * \brief  Function prototypes for SD card R/W 
*/
/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMOSDRW_H_
#define _DEMOSDRW_H_

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void HSMMCSDIntRegister(void);
extern void HSMMCSDContolInit(void);
extern unsigned int HSMMCSDCardPresentStat(void);
extern void HSMMCSDCardAccessSetup(void);

#endif

