/**
 * \file    demoMain.c
 *
 * \brief   This file contains the main() and other functions.
 *
 */

#include "main.h"

#include "demoTimer.h"
#include "enet.h"
#include "demoLedIf.h"
#include "demoRtc.h"
//#include "demoSdRw.h"
//#include "demoSdFs.h"
#include "i2c.h"

#include "interrupt.h"
#include "soc_AM335x.h"
#include "cp15.h"
#include "beaglebone.h"
#include "uartStdio.h"
#include "delay.h"
#include <string.h>

/****************************************************************************
**                   INTERNAL MACRO DEFINITIONS                                       
****************************************************************************/
#define NUM_OF_IMAGES                         (9u)
#define RGN_L2_WBWA                           (8u)
#define SECT_ATTR(NS, nG, S, AP2, TEX, AP, Dom, XN, C, B) \
             (0x02 | (NS << 19) | (nG << 17) | (S <<16) | \
             (AP2 <<15) | (TEX << 12) | (AP << 10) | (Dom << 5)\
             | (XN << 4) | (C << 3) | (B << 2))

#define CACHEABLE_TLB_ATTR   SECT_ATTR(1, 0, 0, 0, 5, 3, 0, 0, 1, 0)
#define NORM_TLB_ATTR        SECT_ATTR(1, 0, 0, 0, 0, 3, 0, 1, 0, 0)
 
/****************************************************************************
**                   LOCAL FUNCTION PROTOTYPES                                
****************************************************************************/
static void EnetStatusCheckNUpdate(void);
static void PeripheralsSetUp(void);
static void ContextReset(void);
static void dummyIsr(void);

/****************************************************************************
**                   EXTERNAL VARIABLE DECLARATIONS                             
****************************************************************************/
extern unsigned int ipAddr;
extern volatile tBoolean bConnected;

/*******************************************************************************
**                     EXTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
extern void etharp_tmr(void);

/****************************************************************************
**                  GLOBAL VARIABLES DEFINITIONS                                         
****************************************************************************/

unsigned char runData[ 32 ];
unsigned int runCommand = 0;

unsigned int clickIdx = 0;

#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment=(16*1024)
static volatile unsigned int pageTable[4*1024];

#elif defined(__TMS470__)
#pragma DATA_ALIGN(pageTable, (16*1024));
static volatile unsigned int pageTable[4*1024];

#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/****************************************************************************
**                      FUNCTION DEFINITIONS                                         
****************************************************************************/
/*
** Enable all the peripherals in use
*/
static void PeripheralsSetUp(void)
{
    DMTimer2ModuleClkConfig();
    DMTimer3ModuleClkConfig();
    DMTimer4ModuleClkConfig();
    RTCModuleClkConfig();
    CPSWPinMuxSetup();
    CPSWClkEnable();
    EDMAModuleClkConfig();
    GPIO1ModuleClkConfig();
    GPIO1Pin23PinMuxSetup();
    HSMMCSDPinMuxSetup();
    HSMMCSDModuleClkConfig();
}

/*
** Resets the state
*/
static void ContextReset(void)
{
    tmrFlag  = FALSE;
    LedOff();
    rtcSetFlag = FALSE;
    rtcSecUpdate = FALSE;
//    sdCardAccessFlag = FALSE;
}

/*
** Main function. The application starts here.
*/
int main(void)
{
    unsigned int index;
    unsigned int j;

    /*
    ** Sets up Section page tables. This is only first level
    ** page table, each page is of size 1MB
    */
    for(index = 0; index < (4*1024); index++)
    {
         /* Set the cacheable memory attributes */
         if((index >= 0x800 && index < 0x880) || (index == 0x403))
         {
              pageTable[index] = (index << 20) | CACHEABLE_TLB_ATTR;
         }

         /* Set the non-cacheable memory attributes */
         else
         {
              pageTable[index] = (index << 20) | NORM_TLB_ATTR;
         }
    }


    /* Invalidate the TLB, pipeline */
    CP15TlbInvalidate();
    CP15BranchPredictorInvalidate();

    CP15BranchPredictionEnable();

    CP15DomainAccessClientSet();

    /* Set TTB0 value. We use only TTB0 here (N = 0) */
    CP15Ttb0Set(((unsigned int )pageTable) | RGN_L2_WBWA);

    /* Enables MMU */
    CP15MMUEnable();

    /* Flush and enable Instruction Cache */
    CP15ICacheFlush();
    CP15ICacheEnable();

    PeripheralsSetUp();

    /* Initialize the ARM Interrupt Controller */
    IntAINTCInit();

    /* Register the ISRs */  
    Timer2IntRegister();
    Timer4IntRegister();
    EnetIntRegister();
    RtcIntRegister();
//    HSMMCSDIntRegister();
    IntRegister(127, dummyIsr);

    IntMasterIRQEnable();

    /* Enable system interrupts */
    IntSystemEnable(SYS_INT_RTCINT);
    IntPrioritySet(SYS_INT_RTCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT2);
    IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT4);
    IntPrioritySet(SYS_INT_TINT4, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_MMCSD0INT);
    IntPrioritySet(SYS_INT_MMCSD0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_EDMACOMPINT);
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(127);
    IntPrioritySet(127, 0, AINTC_HOSTINT_ROUTE_IRQ);

    RtcInit();
    UARTStdioInit();
//    HSMMCSDContolInit();
    DelayTimerSetup();

    Timer2Config();
    Timer4Config();
    LedIfConfig();

    Timer2IntEnable();
    Timer4IntEnable();
    RtcSecIntEnable();

    InitI2C();

    Timer4Start(); 

    for( index = 0; index < 5; ++index )
    {
        LedToggle();

        for( j = 0; j < 100000; ++j );
    }

    // TEMP
    //i2cTest();

    /*
    ** Loop for ever. Necessary actions shall be taken
    ** after detecting the click.
    */
    while( 1 )
    {
        EnetStatusCheckNUpdate();

        if( runCommand )
        {
            if( runData[ 1 ] == 'D' )
            {
                UARTPuts( "** DAC SET\n\r", -1 );

                if( runData[ 2 ] == 'A' )
                {
                    i2cDAC_Set( 0, runData[ 3 ], runData[ 4 ] );
                }
                if( runData[ 5 ] == 'B' )
                {
                    i2cDAC_Set( 1, runData[ 6 ], runData[ 7 ] );
                }
                if( runData[ 8 ] == 'C' )
                {
                    i2cDAC_Set( 2, runData[ 9 ], runData[ 10 ] );
                }
                if( runData[ 11 ] == 'D' )
                {
                    i2cDAC_Set( 3, runData[ 12 ], runData[ 13 ] );
                }
            }
            else if( runData[ 1 ] == 'G' )
            {
                UARTPuts( "** GPIO OFF\n\r", -1 );

                if( runData[ 2 ] == 0 ) // Off
                {
                    i2cGPIO_Off( 0, 1 << runData[ 3 ] );
                }
                else if( runData[ 2 ] == 1 ) // On
                {
                    i2cGPIO_On( 0, 1 << runData[ 3 ] );
                }
            }

            runCommand = 0;
        }

         /*
         ** Check if click is detected
         */
         if(clickIdx != 0)
         {
             /*
             ** Take the Action for click
             */
             clickIdx = 0;
         }
       
         /*
         ** Check if the Timer Expired
         */ 
         if(TRUE == tmrFlag)
         {
             /* Toggle the LED state */
             LedToggle();
             tmrFlag = FALSE;
         }
 
         /*
         ** Check if RTC Time is set
         */
         if(TRUE == rtcSetFlag)
         {
             if(TRUE == rtcSecUpdate)
             { 
                 rtcSecUpdate = FALSE;
                 RtcTimeCalDisplay();
             }
         } 
   
         /*
         ** Check for SD Card
         */
/*
         if(TRUE == sdCardAccessFlag)
         {
             HSMMCSDCardAccessSetup();
         }
*/

         if(TRUE == tmr4Flag)
         {
            tmr4Flag = FALSE;
            etharp_tmr();
         }
    }
}

/*
** Check for any change in ethernet link status. If so, update
** ip address
*/
static void EnetStatusCheckNUpdate(void)
{
    unsigned int linkFlag = FALSE;
    static unsigned int prevEnState = 0;
    static unsigned int nxtEnState = 1;

    if(prevEnState != nxtEnState)
    {
        if(!EnetIfIsUp())
        {
            ContextReset();
            linkFlag = FALSE;
            EnetHttpServerInit();

            if(ipAddr)
            {
                linkFlag = TRUE; 
                prevEnState = 1;
            }
        }
        else
        {
            if(EnetLinkIsUp())
            {
                linkFlag = TRUE;
                nxtEnState = 1;
            }
            else
            {
                ContextReset();
                linkFlag = FALSE;
                prevEnState = 0;
                nxtEnState = 0;
            }
        }

        if((TRUE == linkFlag) && (ipAddr != 0))
        {
             prevEnState = 1;
             UARTPuts("\n\rAccess the home page using http://", -1);
             IpAddrDisplay();
             UARTPuts("/index.html \n\r", -1);
        }
        else
        {
            UARTPuts("\n\rNetwork Connection failed.\n\r", -1);
        }
    }
    else
    {
        if(EnetLinkIsUp())
        {  
            nxtEnState = 1;
        }
        else
        {
            nxtEnState = 0;
        }
    }
}

/*
** Dummy ISR to handle spurious interrupts
*/
static void dummyIsr(void)
{
    ; /* Perform nothing */
}

/****************************** End of file *********************************/
 
