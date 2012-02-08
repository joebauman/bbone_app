/*
 * \file    demoEnet.c
 *
 * \brief   This file contains Ethernet related functions.
 *
*/

/* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */

#include "soc_AM335x.h"
#include "beaglebone.h"
#include "interrupt.h"
#include "demoEnet.h"
#include "demoCfg.h"
#include "lwiplib.h"
#include "net.h"
#include "demoMain.h"
#include "lwipopts.h"
#include "cpsw.h"
#include "uartStdio.h"

#include <string.h>

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define LEN_IP_ADDR                          (4u)
#define ASCII_NUM_IDX                        (48u)
#define NUM_CONFIG_CGI_URIS                  (1)
#define CPSW_STAT_TX_COL                     (0x48)
#define CPSW_STAT_RX_CRC_ERR                 (0x10)
#define CPSW_STAT_RX_ALIGN_CODE              (0x14)

/******************************************************************************
**                     INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void CPSWCore0RxIsr(void);
static void CPSWCore0TxIsr(void);
//static char* ControlCGIHandler(char *params);

/*******************************************************************************
**                     INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
//static const tCGI g_psConfigCGIURIs[] =
//{
//    { "/io_control.cgi", ControlCGIHandler }
//};

unsigned int ipAddr;

/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/* 
** Registers ethernet ISRs
*/
void EnetIntRegister(void)
{
    /* Register the Receive ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWRXINT0, CPSWCore0RxIsr);

    /* Register the Transmit ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWTXINT0, CPSWCore0TxIsr);

    /* Set the priority */
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
}

unsigned int EnetIfIsUp(void)
{
   return (lwIPNetIfStatusGet(0));
}

unsigned int EnetLinkIsUp(void)
{

   return (lwIPLinkStatusGet(0));
}

/*
** initializes the httpserver and prints the IP address on the UART console
*/
void EnetHttpServerInit(void)
{
    unsigned char macArray[6];
    volatile unsigned int cnt = 3;

    EVMPortMIIModeSelect();

    EVMMACAddrGet(0, macArray);

    UARTPuts("\n\rAcquiring IP Address...\n\r", -1);

    ipAddr = 0;

    while(cnt--)
    {  
        
    /* Initialze the lwIP library, using DHCP.*/
#if STATIC_IP_ADDRESS

        ipAddr = lwIPInit(0, macArray, STATIC_IP_ADDRESS, 0, 0, IPADDR_USE_STATIC);

#else

        ipAddr = lwIPInit(0, macArray, 0, 0, 0, IPADDR_USE_DHCP);

#endif
        if( 0!= ipAddr)
        {
            break;
        }
    }


    if(0 == ipAddr)
    {
        UARTPuts("\n\rIP Address not assigned. Check connection/DHCP.\n\r",
                 -1);
        return;
    }
    
    UARTPuts("\n\rEVM IP Address Assigned: ", -1);

    IpAddrDisplay();

//    http_set_cgi_handlers(g_psConfigCGIURIs, NUM_CONFIG_CGI_URIS);

    /* Initialize the sample httpd server. */
    net_init();
}



/*
** CGI handler 
*/
char* ControlCGIHandler(char *params)
{
    if(!(strcmp(params,"TIMER")))
    {
        clickIdx = CLICK_IDX_TIMER;    
    }

    else if(!(strcmp(params,"LED")))
    {
        clickIdx = CLICK_IDX_LED;    
    }

    else if(!(strcmp(params,"RTC")))
    {
        clickIdx = CLICK_IDX_RTC;    
    }

    else if(!(strcmp(params,"MMCSD")))
    {
        clickIdx = CLICK_IDX_SD;
    }
 
    else
    {
        clickIdx = 0;
    }
 
    return "/io_cgi.ssi";
}


/*
** Displays the IP addrss on the UART Console
*/
void IpAddrDisplay(void)
{
    unsigned char byte;
    int cnt;

    for(cnt = 0; cnt <= LEN_IP_ADDR - 1; cnt++)
    {
        byte = (ipAddr >> (cnt * 8)) & 0xFF;

        if(cnt)
        {
            UARTPuts(".", -1);
        }

        UARTPutNum((int)byte);
    }
}

/*
** Interrupt Handler for Core 0 Receive interrupt
*/
static void CPSWCore0RxIsr(void)
{
    UARTPuts( "Enter CPSWCore0RxISR \n\r", -1 );
	lwIPRxIntHandler(0);
    UARTPuts( "Exit CPSWCore0RxISR \n\r", -1 );
}

/*
** Interrupt Handler for Core 0 Transmit interrupt
*/
static void CPSWCore0TxIsr(void)
{
    UARTPuts( "Enter CPSWCore0TxISR \n\r", -1 );
    lwIPTxIntHandler(0);
    UARTPuts( "Exit CPSWCore0TxISR \n\r", -1 );
}

/*
** Returns the Error Statistics
*/
void EnetErrStatsticsGet(unsigned int *tx, unsigned int *rx)
{
    /* Transmit collitions */
    *tx = CPSWStatisticsGet(SOC_CPSW_STAT_REGS, CPSW_STAT_TX_COL);

    /* Receive CRC errors and alignment/code errors */
    *rx = (CPSWStatisticsGet(SOC_CPSW_STAT_REGS, CPSW_STAT_RX_CRC_ERR)
           + CPSWStatisticsGet(SOC_CPSW_STAT_REGS, CPSW_STAT_RX_ALIGN_CODE));
}

/****************************** End of file **********************************/



