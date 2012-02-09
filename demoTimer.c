/*
 * \file    demoTimer.c
 *
 * \brief   This file contains Timer related functions.
 *
*/

/* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */

#include "demoTimer.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "dmtimer.h"


/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TIMER_INITIAL_COUNT             (0xFFE00000u)
#define TIMER_RLD_COUNT                 (0xFFE00000u)
#define TMR_STEP_CNT                    (7u)

/*	1 count at 32.768 KHz takes 31.25us	*/
/*	0x1900 counts takes 200ms */
#define TIMER_200MS_DELAY               (0xFFFFE6FF)
/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void Timer2Isr(void);
static void Timer4Isr(void);

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int tmrFlag = FALSE;
unsigned int tmrClick = FALSE;
volatile  unsigned int tmr4Flag = FALSE;

unsigned int timerCount[10] = 
						{
							0xFFF00000u,
							0xFFE00000u,
							0xFFC00000u,
							0xFFA00000u,
							0xFF800000u,
							0xFF600000u,
							0xFF400000u,
							0xFF200000u,
							0xFF000000u,
							0xFD000000u,
						};

/*******************************************************************************
**                     FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Registers the Timer2 ISR.
*/
void Timer2IntRegister(void)
{
    IntRegister(SYS_INT_TINT2, Timer2Isr);
}

/*
** Configures the Timer2 for 32 bit
*/
void Timer2Config(void)
{
    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    //DMTimerReloadSet(SOC_DMTIMER_2_REGS, TIMER_RLD_COUNT);

    /* Configure the DMTimer for one shot mode */
    DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_ONESHOT_NOCMP_ENABLE);
    tmrFlag = FALSE;
	Timer2Stop();
}

/*
** Enables the Timer2 Interrupts
*/
void Timer2IntEnable(void)
{
    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

/*
** Starts the Timer
*/
void Timer2Start(void)
{
    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_2_REGS);
}

/*
** Stops the Timer. The Timer Counter is Reset.
*/
void Timer2Stop(void)
{
    DMTimerDisable(SOC_DMTIMER_2_REGS);
    //DMTimerCounterSet(SOC_DMTIMER_2_REGS, 0);
}

/*
** Timer 2 Interrupt Service Routine
*/
static void Timer2Isr(void)
{
	static unsigned int index = 0;
	
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
 
    /* Notify end of interrupt */
    DMTimerEndOfInterrupt(SOC_DMTIMER_2_REGS);

    if(TRUE == tmrClick)
    {
        tmrFlag = TRUE;
    }
    
    else 
    {
        tmrFlag = FALSE;
    }
	
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, timerCount[index++%10]);
	
	DMTimerEnable(SOC_DMTIMER_2_REGS);	
	
}



/*******************************************************************************
**                     FUNCTION DEFINITIONS TIMER4
*******************************************************************************/
/*
** Registers the Timer4 ISR.
*/
void Timer4IntRegister(void)
{
    IntRegister(SYS_INT_TINT4, Timer4Isr);
	  
    /* Set the priority */
    IntPrioritySet(SYS_INT_TINT4, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_TINT4);
}

/*
** Configures the Timer4 for 32 bit
*/
void Timer4Config(void)
{
    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_4_REGS, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    DMTimerReloadSet(SOC_DMTIMER_4_REGS, TIMER_200MS_DELAY);

    /* Configure the DMTimer for one shot mode */
    DMTimerModeConfigure(SOC_DMTIMER_4_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);
	
	Timer4Stop();
}

/*
** Enables the Timer4 Interrupts
*/
void Timer4IntEnable(void)
{
    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);

}

/*
** Starts the Timer
*/
void Timer4Start(void)
{
    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_4_REGS);
}

/*
** Stops the Timer. The Timer Counter is Reset.
*/
void Timer4Stop(void)
{
    DMTimerDisable(SOC_DMTIMER_4_REGS);
    //DMTimerCounterSet(SOC_DMTIMER_4_REGS, 0);
}

/*
** Timer 4 Interrupt Service Routine
*/
static void Timer4Isr(void)
{
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
    
    tmr4Flag = TRUE;
	
	//DMTimerEnable(SOC_DMTIMER_4_REGS);	
	
}

/******************************** End of file **********************************/

