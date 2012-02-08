/*
 * \file    demoTimer.c
 *
 * \brief   This file contains Timer related functions.
 *
*/

/* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */
#include "soc_AM335x.h"
#include "interrupt.h"
#include "evmAM335x.h"
#include "uartStdio.h"
#include "rtc.h"
#include "demoRtc.h"

/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define MASK_HOUR                        (0xFF000000u)
#define MASK_MIN                         (0x00FF0000u)
#define MASK_SEC                         (0x0000FF00u)
#define MASK_MERIDIEM                    (0x000000FFu)

#define SHIFT_HOUR                       (24u)
#define SHIFT_MIN                        (16u)
#define SHIFT_SEC                        (8u)

#define MASK_DAY                         (0xFF000000u)
#define MASK_MON                         (0x00FF0000u)
#define MASK_YEAR                        (0x0000FF00u)
#define MASK_DOTW                        (0x000000FFu)

#define SHIFT_DAY                        (24u)
#define SHIFT_MON                        (16u)
#define SHIFT_YEAR                       (8u)

/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void RTCIsr(void);

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
unsigned int rtcSetFlag = FALSE;
unsigned int rtcSecUpdate = FALSE;

/*******************************************************************************
**                     FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Registers RTC interrupts
*/
void RtcIntRegister(void)
{
    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_RTCINT, RTCIsr);
}

/*
** Enables RTC seconds interrupt
*/
void RtcSecIntEnable(void)
{
    /* Enable interrupts to be generated on every second.*/
    RTCIntTimerEnable(SOC_RTC_0_REGS, RTC_INT_EVERY_SECOND);
}

/*
** Initializes the RTC peripheral
*/
void RtcInit(void)
{
    /* Disabling Write Protection for RTC registers.*/
    RTCWriteProtectDisable(SOC_RTC_0_REGS);

    /* Selecting Internal Clock source for RTC. */
    RTC32KClkSourceSelect(SOC_RTC_0_REGS, RTC_INTERNAL_CLK_SRC_SELECT);

    /* Enabling RTC to receive the Clock inputs. */
    RTC32KClkClockControl(SOC_RTC_0_REGS, RTC_32KCLK_ENABLE);

//    RTCEnable(SOC_RTC_0_REGS);
    RTCDisable( SOC_RTC_0_REGS );
}

/*
** Sets the Time and Calender in the RTC. This is a blocking call. 
** The time and date are entered through UART.
*/
void RtcTimeCalSet(void)
{
    unsigned int time = 0; 
    unsigned int cal = 0;
    unsigned int temp = 0; 
 
    UARTPuts("\n\rEnter Hours (0 to 23):", -1);
    temp = UARTGetNum();

    while(temp > 23)
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    time = (((temp / 10) << 4) << SHIFT_HOUR)
            | ((temp % 10) << SHIFT_HOUR);

    UARTPuts("\n\rEnter Minutes (0 to 59):", -1);
    temp = UARTGetNum();

    while(temp > 59)
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    time |= (((temp / 10) << 4) << SHIFT_MIN)
            | ((temp % 10) << SHIFT_MIN);
 
    UARTPuts("\n\rEnter Seconds (0 to 59):", -1);
    temp = UARTGetNum();

    while(temp > 59)
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    time |= (((temp / 10) << 4) << SHIFT_SEC)
             | ((temp % 10) << SHIFT_SEC);

    UARTPuts("\n\rEnter Date (1 to 31):", -1);
    temp = UARTGetNum();

    while((temp > 31) || (0 == temp))
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    cal = (((temp / 10) << 4) << SHIFT_DAY)
           | ((temp % 10) << SHIFT_DAY);

    UARTPuts("\n\rEnter Month (1 to 12):", -1);
    temp = UARTGetNum();

    while((temp > 12) || (0 == temp))
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    cal |= (((temp / 10) << 4) << SHIFT_MON)
            | ((temp % 10) << SHIFT_MON);

    UARTPuts("\n\rEnter Year (0 to 99):", -1);
    temp = UARTGetNum();
    while(temp > 99)
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    cal |= (((temp / 10) << 4) << SHIFT_YEAR)
            | ((temp % 10) << SHIFT_YEAR);

    UARTPuts("\n\rEnter Day Of the week (0 for Sunday...6 for Saturday):", -1);
    temp = UARTGetNum();

    while(temp > 6)
    {
        UARTPuts("\n\rValue entered is invalid. Enter value:", -1);
        temp = UARTGetNum();
    }

    cal |= (((temp / 10) << 4)) | ((temp % 10));
 
    /* Set the calendar registers of RTC with received calendar information.*/
    RTCCalendarSet(SOC_RTC_0_REGS, cal);

    /* Set the time registers of RTC with the received time information.*/
    RTCTimeSet(SOC_RTC_0_REGS, time);

    /* Run the RTC. The seconds tick from now on.*/
    RTCRun(SOC_RTC_0_REGS);
 
    UARTPuts("\n\rThe Time and Date are set successfully! \n\n\r", -1);

    rtcSetFlag = TRUE;
}

/*
** Displays the Time and Date on the UART console
*/
void RtcTimeCalDisplay(void)
{
    unsigned int time = 0;
    unsigned int cal = 0;
    unsigned int temp;
 
    UARTPuts("\r", -1);

    time = RTCTimeGet(SOC_RTC_0_REGS);
    UARTPuts("Current Time And Date: ", -1);
 
    temp = (time & MASK_HOUR) >> SHIFT_HOUR;
    UARTPutc(((temp >> 4) & 0x0F) + 48);
    UARTPutc((temp & 0x0F) + 48);
  
    UARTPutc(':');
 
    temp = (time & MASK_MIN) >> SHIFT_MIN;
    UARTPutc(((temp >> 4) & 0x0F) + 48);
    UARTPutc((temp & 0x0F) + 48);

    UARTPutc(':');

    temp = (time & MASK_SEC) >> SHIFT_SEC;
    UARTPutc(((temp >> 4) & 0x0F) + 48);
    UARTPutc((temp & 0x0F) + 48);

    UARTPuts(", ", -1);

    cal = RTCCalendarGet(SOC_RTC_0_REGS);

    temp = (cal & MASK_DAY) >> SHIFT_DAY;
    UARTPutc(((temp >> 4) & 0x0F) + 48);
    UARTPutc((temp & 0x0F) + 48);

    UARTPutc('-');

    temp = (cal & MASK_MON) >> SHIFT_MON;
    UARTPutc(((temp >> 4) & 0x0F) + 48);
    UARTPutc((temp & 0x0F) + 48);

    UARTPutc('-');

    temp = (cal & MASK_YEAR) >> SHIFT_YEAR;
    UARTPutc(((temp >> 4) & 0x0F) + 48);
    UARTPutc((temp & 0x0F) + 48);

    UARTPuts(", ", -1);

    switch(cal & MASK_DOTW)
    {
        case 0x00:
             UARTPuts("Sunday", -1);
        break;

        case 0x01:
             UARTPuts("Monday", -1);
        break;

        case 0x02:
             UARTPuts("Tuesday", -1);
        break;

        case 0x03:
             UARTPuts("Wednesday", -1);
        break;

        case 0x04:
             UARTPuts("Thursday", -1);
        break;

        case 0x05:
             UARTPuts("Friday", -1);
        break;

        case 0x06:
             UARTPuts("Saturday", -1);

        default:
        break;

    }
}

/*
** Interrupt service routine for RTC
*/
static void RTCIsr(void)
{
    rtcSecUpdate = TRUE;
}

/******************************** End of file **********************************/



