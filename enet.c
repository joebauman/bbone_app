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
#include "enet.h"
#include "lwiplib.h"
#include "net.h"
#include "main.h"
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

/*******************************************************************************
**                     INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
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
void EnetHttpServerInit( unsigned int ip )
{
    unsigned char macArray[6];
    volatile unsigned int cnt = 3;

    EVMPortMIIModeSelect();

    EVMMACAddrGet(0, macArray);

    UARTPuts("\n\rAcquiring IP Address...\n\r", -1);

    ipAddr = 0;

    while(cnt--)
    {  
        
        /* Initialze the lwIP library, using DHCP or STATIC.*/
        if( ip > 0 )
        {
            ipAddr = lwIPInit( 0, macArray, ip, 0, 0, IPADDR_USE_STATIC );
        }
        else
        {
            ipAddr = lwIPInit( 0, macArray, 0, 0, 0, IPADDR_USE_DHCP );
        }

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

    // Initialize the socket server.
    net_init();
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
    lwIPRxIntHandler(0);
}

/*
** Interrupt Handler for Core 0 Transmit interrupt
*/
static void CPSWCore0TxIsr(void)
{
    lwIPTxIntHandler(0);
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



