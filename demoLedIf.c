/*
 * \file    demoLedIf.c
 *
 * \brief   This file contains LED interface related functions.
 *
*/

/* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */

#include "soc_AM335x.h"
#include "beaglebone.h"
#include "demoCfg.h"
#include "gpio_v2.h"
#include "demoLedIf.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define LED_TOGGLE                           (0x01u)
#define LED_OFF                              (GPIO_PIN_LOW)

/*******************************************************************************
**                     INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static unsigned char ledState = GPIO_PIN_LOW;

/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Configures the I2C for the LED interface
*/
void LedIfConfig(void)
{
    /* Enabling the GPIO module. */
    GPIOModuleEnable(SOC_GPIO_1_REGS);

    /* Resetting the GPIO module. */
    GPIOModuleReset(SOC_GPIO_1_REGS);

    /* Setting the GPIO pin as an output pin. */
    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_DIR_OUTPUT);
}

/*
** Toggle the LED state
*/
void LedToggle(void)
{
    ledState ^= LED_TOGGLE;  
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, ledState); 
}

/*
** Turn the  LED Off.
*/
void LedOff(void)
{
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
}
/*
** Turn the  LED Off.
*/
void LedOn(void)
{
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
}

/****************************** End of file **********************************/



