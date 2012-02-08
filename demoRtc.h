/*
 * \file   demoRtc.h
 *
 * \brief  Function prototypes for the RTC to set time and date
*/
/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMORTC_H_
#define _DEMORTC_H_

/******************************************************************************
**                     EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern unsigned int rtcSetFlag;
extern unsigned int rtcSecUpdate;

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void RtcIntRegister(void);
extern void RtcSecIntEnable(void);
extern void RtcInit(void);
extern void RtcTimeCalSet(void);
extern void RtcTimeCalDisplay(void);

#endif
