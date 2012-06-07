/*
 * \file   demoEnet.h
 *
 * \brief  Function prototypes for the Ethernet 
*/
/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMOENET_H_
#define _DEMOENET_H_

/******************************************************************************
**                     EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void EnetIntRegister(void);
extern void EnetHttpServerInit( unsigned int ip );
extern void IpAddrDisplay(void);
extern unsigned int EnetIfIsUp(void);
extern unsigned int EnetLinkIsUp(void);
extern void EnetErrStatsticsGet(unsigned int *tx, unsigned int *rx);

#endif
