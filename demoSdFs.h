/*
 * \file   demoSdFs.h
 *
 * \brief  Function prototypes for SD card file system access
*/
/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMOSDFS_H_
#define _DEMOSDFS_H_

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern volatile unsigned int sdCardAccessFlag;

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void HSMMCSDFsMount(unsigned int driveNum, void *ptr);
extern void HSMMCSDFsProcessCmdLine(void);

#endif

