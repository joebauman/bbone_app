
#include "i2c.h"

#include "hsi2c.h"

#include "beaglebone.h"
//#include "evmAM335x.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "soc_AM335x.h"
#include "lwiplib.h"
#include "lwipopts.h"
#include "delay.h"


/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int tCount;
volatile unsigned int rCount;
volatile unsigned int flag = 1;
volatile unsigned int numOfBytes;
volatile unsigned char dataToSlave[ 16 ];
volatile unsigned char dataFromSlave[ 50 ];

// Variables to hold the current hardware state
static unsigned char GPIO_Values[ 2 ];

static unsigned char DAC_Values[ 2 * 4 ]; // 2 bytes * 4 channels

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

 /* 
 ** Reads data from a specific address of e2prom
 */
void E2promRead(unsigned char *data)
{
    unsigned int i;

    dataToSlave[0] = E2PROM_ADDR_MSB;
    dataToSlave[1] = E2PROM_ADDR_LSB;

    tCount = 0;
    rCount = 0;
    SetupI2CReception(0, 2, 50);

    for (i = 0; i < 50; i++ )
    {
        data[i] = dataFromSlave[i];
    }
}

void InitI2C( void )
{
    unsigned int i;

    // I2C Interrupts
    IntRegister(SYS_INT_I2C0INT, I2C0Isr);
    IntRegister(SYS_INT_I2C1INT, I2C1Isr);

    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntPrioritySet(SYS_INT_I2C1INT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    IntSystemEnable(SYS_INT_I2C0INT);
    IntSystemEnable(SYS_INT_I2C1INT);

    // Initialize variables
    GPIO_Values[ 0 ] = 0;
    GPIO_Values[ 1 ] = 0;

    for( i = 0; i < 8; ++i )
    {
        DAC_Values[ i ] = 0;
    }

    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = GPIO_Values[ 0 ];
    dataToSlave[ 1 ] = GPIO_Values[ 1 ];

    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    // Setup the UART on the V5 board.
    SetupI2C( 1, I2C_ADDR_V5_UART );

    // Set the baud rate: Baud rate is defined in i2c.h file
    // 115200: DLL = 0x08, DLH = 0x00
    // 9600 (Zaber): DLL = 0x60, DLH = 0x00
    // Enable Divisor Latch
    dataToSlave[ 0 ] = reg_LCR_ADDR;
    dataToSlave[ 1 ] = 0x80;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    // Write to DLL and DLH
    if( baudRate == 115200 )
    {
        dataToSlave[ 1 ] = 0x08;
    }
    else
    {
        dataToSlave[ 1 ] = 0x60;
    }

    dataToSlave[ 0 ] = reg_DLL_ADDR;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = reg_DLH_ADDR;
    dataToSlave[ 1 ] = 0x00;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    // Lock the Division Latch
    dataToSlave[ 0 ] = reg_LCR_ADDR;
    dataToSlave[ 1 ] = 0x03;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    // Write to FCR register to clear and enable fifos
    dataToSlave[ 0 ] = reg_FCR_ADDR;
    dataToSlave[ 1 ] = 0x07;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}

void SetupI2C( unsigned int channel, unsigned int slaveAddr )
{
    //
    // I2CN Setup
    //

    unsigned int regBase = 0;

    if( channel == 0 ) regBase = SOC_I2C_0_REGS;
    else if( channel == 1 ) regBase = SOC_I2C_1_REGS;
    else if( channel == 2 ) regBase = SOC_I2C_2_REGS;
    else regBase = SOC_I2C_0_REGS;

    /* Enable the clock for I2CN */
    I2C0ModuleClkConfig();
    I2C1ModuleClkConfig();
    //I2C2ModuleClkConfig();

    I2CPinMuxSetup( channel );

    /* Put i2c in reset/disabled state */
    I2CMasterDisable( regBase );

    /* Disable auto Idle functionality */
    I2CAutoIdleDisable( regBase );

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk( regBase, 48000000, 12000000, 100000 );

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet( regBase, slaveAddr);

    /* Bring I2C out of reset */
    I2CMasterEnable( regBase );
}

/*
** Transmits data over I2C bus 
*/
void SetupI2CTransmit( unsigned int channel, unsigned int dcount )
{
    unsigned int regBase = 0;

    if( channel == 0 ) regBase = SOC_I2C_0_REGS;
    else if( channel == 1 ) regBase = SOC_I2C_1_REGS;
    else if( channel == 2 ) regBase = SOC_I2C_2_REGS;
    else regBase = SOC_I2C_0_REGS;

    /* Data Count specifies the number of bytes to be transferred */
    I2CSetDataCount(regBase, dcount);

    numOfBytes = I2CDataCountGet(regBase);


    CleanUpInterrupts( regBase );

    /* 
    ** Configure I2C controller in Master Transmitter mode.A stop
    ** condition will be generated after data count number of
    ** bytes are transferred.
    */
    I2CMasterControl(regBase, I2C_CFG_MST_TX | I2C_CFG_STOP);

    /* Transmit and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(regBase, I2C_INT_TRANSMIT_READY |
                                         I2C_INT_STOP_CONDITION );

    /* Generated Start Condition over I2C bus */
    I2CMasterStart(regBase);

    while(flag);

    /* Wait untill I2C registers are ready to access */
    while(0 == (I2CMasterIntRawStatus(regBase) & I2C_INT_ADRR_READY_ACESS));

    flag = 1;
}

/*
** Receives data over I2C bus 
*/
void SetupI2CReception( unsigned int channel,
                               unsigned int xcount, unsigned int dcount )
{
    unsigned int regBase = 0;

    if( channel == 0 ) regBase = SOC_I2C_0_REGS;
    else if( channel == 1 ) regBase = SOC_I2C_1_REGS;
    else if( channel == 2 ) regBase = SOC_I2C_2_REGS;
    else regBase = SOC_I2C_0_REGS;

    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(regBase, xcount);

    numOfBytes = I2CDataCountGet(regBase);

    /* Clear status of all interrupts */
    CleanUpInterrupts( regBase );

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(regBase, I2C_CFG_MST_TX);

    /* Transmit interrupt is enabled */
    I2CMasterIntEnableEx(regBase, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(regBase);

    while(I2CMasterBusBusy(regBase) == 0);

    while(tCount != numOfBytes);

    flag = 1;

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(regBase) & (I2C_INT_ADRR_READY_ACESS)));

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(regBase, dcount);

    numOfBytes = I2CDataCountGet(regBase);

    CleanUpInterrupts( regBase );

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(regBase, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(regBase,  I2C_INT_RECV_READY | I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(regBase);

    while(I2CMasterBusBusy(regBase) == 0);

    while(flag);

    flag = 1;
}


/* Clear status of all interrupts */
void CleanUpInterrupts( unsigned int regBase )
{
    I2CMasterIntEnableEx( regBase, 0x7FF );
    I2CMasterIntClearEx( regBase,  0x7FF );
    I2CMasterIntDisableEx( regBase, 0x7FF );
}

/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
void I2C0Isr(void)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(SOC_I2C_0_REGS);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(SOC_I2C_0_REGS,
                        (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
                        
    if(status & I2C_INT_RECV_READY)
    {
        /* Receive data from data receive register */
        dataFromSlave[rCount++] = I2CMasterDataGet(SOC_I2C_0_REGS);

        /* Clear receive ready interrupt status */    
        I2CMasterIntClearEx(SOC_I2C_0_REGS,  I2C_INT_RECV_READY);
        
        if(rCount == numOfBytes)
        {
            /* Disable the receive ready interrupt */
            I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_RECV_READY);
            /* Generate a STOP */
            I2CMasterStop(SOC_I2C_0_REGS);
        }
    }

    if (status & I2C_INT_TRANSMIT_READY)
    {
        /* Put data to data transmit register of i2c */
        I2CMasterDataPut(SOC_I2C_0_REGS, dataToSlave[tCount++]);

        /* Clear Transmit interrupt status */
        I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);         
                        
        if(tCount == numOfBytes)
        {
            /* Disable the transmit ready interrupt */
            I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);
        }
    }
        
    if (status & I2C_INT_STOP_CONDITION)
    {
        /* Disable transmit data ready and receive data read interupt */
        I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
                                              I2C_INT_RECV_READY     |
                                              I2C_INT_STOP_CONDITION);
        flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
        I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY  |
                                              I2C_INT_RECV_READY      |
                                              I2C_INT_NO_ACK          |
                                              I2C_INT_STOP_CONDITION);
        /* Generate a STOP */
        I2CMasterStop(SOC_I2C_0_REGS);

        flag = 0;
    }

    I2CEndOfInterrupt(SOC_I2C_0_REGS, 0);
}

void I2C1Isr(void)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(SOC_I2C_1_REGS);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(SOC_I2C_1_REGS,
                        (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
                        
    if(status & I2C_INT_RECV_READY)
    {
        /* Receive data from data receive register */
        dataFromSlave[rCount++] = I2CMasterDataGet(SOC_I2C_1_REGS);

        /* Clear receive ready interrupt status */    
        I2CMasterIntClearEx(SOC_I2C_1_REGS,  I2C_INT_RECV_READY);
        
        if(rCount == numOfBytes)
        {
            /* Disable the receive ready interrupt */
            I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_RECV_READY);
            /* Generate a STOP */
            I2CMasterStop(SOC_I2C_1_REGS);
        }
    }

    if (status & I2C_INT_TRANSMIT_READY)
    {
        /* Put data to data transmit register of i2c */
        I2CMasterDataPut(SOC_I2C_1_REGS, dataToSlave[tCount++]);

        /* Clear Transmit interrupt status */
        I2CMasterIntClearEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);         
                        
        if(tCount == numOfBytes)
        {
            /* Disable the transmit ready interrupt */
            I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);
        }
    }
        
    if (status & I2C_INT_STOP_CONDITION)
    {
        /* Disable transmit data ready and receive data read interupt */
        I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY |
                                              I2C_INT_RECV_READY     |
                                              I2C_INT_STOP_CONDITION);
        flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
        I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY  |
                                              I2C_INT_RECV_READY      |
                                              I2C_INT_NO_ACK          |
                                              I2C_INT_STOP_CONDITION);
        /* Generate a STOP */
        I2CMasterStop(SOC_I2C_1_REGS);

        flag = 0;
    }

    I2CEndOfInterrupt(SOC_I2C_1_REGS, 0);
}

void i2cTest()
{
    unsigned char data[ 6 ];

    data[ 0 ] = 0;
    data[ 1 ] = 2; // Renumber
    data[ 2 ] = 0;
    data[ 3 ] = 0;
    data[ 4 ] = 0;
    data[ 5 ] = 0;

    i2cUART_Send( data, 6 );

    while( 1 )
    {
        // Send to UART
        data[ 0 ] = 1;
        data[ 1 ] = 20; // Move abs
        data[ 2 ] = 0;
        data[ 3 ] = 0;
        data[ 4 ] = 0;
        data[ 5 ] = 0;

        i2cUART_Send( data, 6 );
    }
}

// Use masks to turn GPIO pins on and off.

void i2cGPIO_On( unsigned char b2, unsigned char b1 )
{
    // Update the local value
    GPIO_Values[ 0 ] |= b1;
    GPIO_Values[ 1 ] |= b2;

    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = GPIO_Values[ 0 ];
    dataToSlave[ 1 ] = GPIO_Values[ 1 ];

    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}

void i2cGPIO_Off( unsigned char b2, unsigned char b1 )
{
    // Update the local value
    GPIO_Values[ 0 ] &= ~b1;
    GPIO_Values[ 1 ] &= ~b2;

    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = GPIO_Values[ 0 ];
    dataToSlave[ 1 ] = GPIO_Values[ 1 ];

    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}

void i2cGPIO_Get( unsigned char *data )
{
    if( data != NULL )
    {
        data[ 0 ] = GPIO_Values[ 0 ];
        data[ 1 ] = GPIO_Values[ 1 ];
    }
}

// Send a dac value.  Chan is 0-3.
void i2cDAC_Set( int chan, unsigned char b1, unsigned char b2 )
{
    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_DAC );

    dataToSlave[ 0 ] = 0x10 | ( ( chan & 0x03 ) << 1 );
    dataToSlave[ 1 ] = b1;
    dataToSlave[ 2 ] = b2;

    tCount = 0;
    SetupI2CTransmit( 1, 3 );

    DAC_Values[ chan * 2 + 0 ] = b1;
    DAC_Values[ chan * 2 + 1 ] = b2;
}

// Return a dac value.  Chan is 0-3.
void i2cDAC_Get( int chan, unsigned char *data )
{
    if( data != NULL )
    {
        data[ 0 ] = DAC_Values[ chan * 2 + 0 ];
        data[ 1 ] = DAC_Values[ chan * 2 + 1 ];
    }
}

void i2cUART_Send( unsigned char *data, unsigned int len )
{
    unsigned int i = 0;

    if( len > 15 ) return;

    SetupI2C( 1, I2C_ADDR_V5_UART );

    dataToSlave[ 0 ] = reg_TX_ADDR;

    for( i = 0; i < len; ++i )
    {
        dataToSlave[ i + 1 ] = data[ i ];
    }

    tCount = 0;
    SetupI2CTransmit( 1, len + 1 );
}

int i2cUART_Recv( unsigned char *data, unsigned int len )
{
    unsigned int i;

    SetupI2C( 1, I2C_ADDR_V5_UART );
    
    // Read RXLVL to see if there is data to receive.
    dataToSlave[ 0 ] = reg_RXLVL_ADDR;

    tCount = 0;
    rCount = 0;
    SetupI2CReception( 1, 1, 1 );

    if( dataFromSlave[ 0 ] < 6 )
    {
        return 0;
    }

    // Read the data
    dataToSlave[ 0 ] = reg_RX_ADDR;

    tCount = 0;
    rCount = 0;
    SetupI2CReception( 1, 1, 6 );

    for( i = 0; i < 6; i++ )
    {
        data[ i ] = dataFromSlave[ i ];
    }

    return 6;
}

