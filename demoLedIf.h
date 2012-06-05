/*
 * \file   demoLedIf.h
 *
 * \brief  Function prototypes for the LED interface
*/
/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMOLEDIF_H_
#define _DEMOLEDIF_H_

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/

#define USER_LED_1  21
#define USER_LED_2  22
#define USER_LED_3  23
#define USER_LED_4  24

extern void LedIfConfig(void);

extern void LedToggle(void);

extern void LedOff( unsigned int led );
extern void LedOn( unsigned int led );

#endif
