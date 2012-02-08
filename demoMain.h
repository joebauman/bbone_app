/*
*  Copyright (C) 2005-2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

#ifndef _DEMOMAIN_H_
#define _DEMOMAIN_H_

/******************************************************************************
**                            TYPE DEFINITIONS
*******************************************************************************/
/*
** Touch Specifications of an image.
*/
typedef struct touchSpec
{
    int const *coOrd;
    void (*action)(void);
}TOUCHSPEC;

/*
** Context of an Image
*/
typedef struct imageContext
{
    unsigned int const *pImageAddr;
    
    /* The number of icons in the image */
    unsigned int numIcon; 

    TOUCHSPEC const *touchSpec;
}IMAGECONTEXT;

/******************************************************************************
**                      EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern IMAGECONTEXT contextInfo[];
extern volatile unsigned int imageCount;
extern unsigned int clickIdx;
extern unsigned int enetInitFlag;

#endif
