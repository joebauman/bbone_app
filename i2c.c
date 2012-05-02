
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

static unsigned char GPIO_Vals[ 2 ];

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

#if 0
    // Send set output direction
    SetupI2C( 1, I2C_ADDR_PIC_EXPAND );

    dataToSlave[ 0 ] = 0x00;
    dataToSlave[ 1 ] = 0x00;

    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    temp = 0xAA;

    while(1)
    {
        SetupI2C( 1, I2C_ADDR_PIC_EXPAND );

        temp ^= 0xFF;

        // Send output
        dataToSlave[ 0 ] = 0x09;
        dataToSlave[ 1 ] = temp;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        // Send output
        //dataToSlave[ 0 ] = 0x0A;
        //dataToSlave[ 1 ] = temp;

        //tCount = 0;
        //SetupI2CTransmit( 2 );

        // Send DAC Voltage
        SetupI2C( 1, I2C_ADDR_PIC_DAC );

        dataToSlave[ 0 ] = 0x00;
        dataToSlave[ 1 ] = temp;
        dataToSlave[ 2 ] = 0x00;

        tCount = 0;
        SetupI2CTransmit( 1, 3 );

        // Read temp sensor
        SetupI2C( 1, I2C_ADDR_PIC_TEMP );

        tempSensorRead( dataRead );

        UARTPutHexNum( dataRead[ 0 ] );
        UARTPutc( ' ' );
        UARTPutHexNum( dataRead[ 1 ] );
        UARTPutc( '\n' );
        UARTPutc( '\r' );

        // Delay
        for( i = 0; i < 500000; ++i );
    }
}
#endif // 0

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

void tempSensorRead( unsigned char *data )
{
    unsigned int i;

    dataToSlave[0] = 0x00; // Ambient value is Reg 0

    tCount = 0;
    rCount = 0;
    SetupI2CReception( 1, 1, 2 );

    for( i = 0; i < 2; i++ )
    {
        data[i] = dataFromSlave[i];
    }
}

void expanderSend( unsigned char data )
{
    SetupI2C( 1, I2C_ADDR_PIC_EXPAND );

    // Send output
    dataToSlave[ 0 ] = 0x09;
    dataToSlave[ 1 ] = data;

    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}

void InitI2C( void )
{
    // I2C Interrupts
    IntRegister(SYS_INT_I2C0INT, I2C0Isr);
    IntRegister(SYS_INT_I2C1INT, I2C1Isr);

    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntPrioritySet(SYS_INT_I2C1INT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    IntSystemEnable(SYS_INT_I2C0INT);
    IntSystemEnable(SYS_INT_I2C1INT);

    // Initialize variables
    GPIO_Vals[ 0 ] = 0;
    GPIO_Vals[ 1 ] = 0;

    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = GPIO_Vals[ 0 ];
    dataToSlave[ 1 ] = GPIO_Vals[ 1 ];

    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    // Send set expander output direction
/*
    SetupI2C( 1, I2C_ADDR_PIC_EXPAND );

    dataToSlave[ 0 ] = 0x00;
    dataToSlave[ 1 ] = 0x00;

    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    expanderSend( 0x00 );
*/

    // Send set uart IO direction
/*
    SetupI2C( 1, I2C_ADDR_CUSTOM_UART0 );
    dataToSlave[ 0 ] = 0x0A << 3;
    dataToSlave[ 1 ] = 0xFF;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    SetupI2C( 1, I2C_ADDR_CUSTOM_UART1 );
    dataToSlave[ 0 ] = 0x0A << 3;
    dataToSlave[ 1 ] = 0xFF;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    // Send uart baud divisors
    SetupI2C( 1, I2C_ADDR_CUSTOM_UART0 );
    dataToSlave[ 0 ] = 0x03 << 3;
    dataToSlave[ 1 ] = 0x80;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = 0x00 << 3;
    dataToSlave[ 1 ] = 0x08; // Div 8 == 115k
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = 0x01 << 3;
    dataToSlave[ 1 ] = 0x00;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = 0x03 << 3;
    dataToSlave[ 1 ] = 0x13;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    SetupI2C( 1, I2C_ADDR_CUSTOM_UART1 );
    dataToSlave[ 0 ] = 0x03 << 3;
    dataToSlave[ 1 ] = 0x80;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = 0x00 << 3;
    dataToSlave[ 1 ] = 0x08; // Div 8 == 115k
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = 0x01 << 3;
    dataToSlave[ 1 ] = 0x00;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );

    dataToSlave[ 0 ] = 0x03 << 3;
    dataToSlave[ 1 ] = 0x13;
    tCount = 0;
    SetupI2CTransmit( 1, 2 );
*/
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
    static unsigned char val = 0xAA;
    unsigned int j;
    static unsigned char letter0 = 'a', letter1 = 'A';

    while( 1 )
    {
        val ^= 0xFF;

        // Send to expander 0
        SetupI2C( 1, I2C_ADDR_CUSTOM_EXP0 );

        dataToSlave[ 0 ] = val;
        dataToSlave[ 1 ] = val;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        // Send to expander 1
        SetupI2C( 1, I2C_ADDR_CUSTOM_EXP1 );

        dataToSlave[ 0 ] = val;
        dataToSlave[ 1 ] = val;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        // Send to UART 0
        SetupI2C( 1, I2C_ADDR_CUSTOM_UART0 );

        // - Toggle IO
        dataToSlave[ 0 ] = 0x0B << 3;
        dataToSlave[ 1 ] = val;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        // - Send serial byte
        dataToSlave[ 0 ] = 0x00;
        dataToSlave[ 1 ] = letter0;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        // Send to UART 1
        SetupI2C( 1, I2C_ADDR_CUSTOM_UART1 );

        // - Toggle IO
        dataToSlave[ 0 ] = 0x0B << 3;
        dataToSlave[ 1 ] = val;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        // - Send serial byte
        dataToSlave[ 0 ] = 0x00;
        dataToSlave[ 1 ] = letter1;

        tCount = 0;
        SetupI2CTransmit( 1, 2 );

        if( ++letter0 == 'z' ) letter0 = 'a';
        if( ++letter1 == 'Z' ) letter1 = 'A';

        for( j = 0; j < 10000; ++j );
    }
}

// Use masks to turn GPIO pins on and off.

void i2cGPIO_On( unsigned char b2, unsigned char b1 )
{
    // Update the local value
    GPIO_Vals[ 0 ] |= b1;
    GPIO_Vals[ 1 ] |= b2;

    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = GPIO_Vals[ 0 ];
    dataToSlave[ 1 ] = GPIO_Vals[ 1 ];

    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}

void i2cGPIO_Off( unsigned char b2, unsigned char b1 )
{
    // Update the local value
    GPIO_Vals[ 0 ] &= ~b1;
    GPIO_Vals[ 1 ] &= ~b2;

    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = GPIO_Vals[ 0 ];
    dataToSlave[ 1 ] = GPIO_Vals[ 1 ];

    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}


// Send a dac value.
void i2cDAC_Set( int chan, unsigned char b2, unsigned char b1 )
{
    // Send to expander 0
    SetupI2C( 1, I2C_ADDR_V5_EXP );

    dataToSlave[ 0 ] = 0x10 | ( ( chan & 0x03 ) << 1 );
    dataToSlave[ 1 ] = b1;
    dataToSlave[ 2 ] = b2;

    tCount = 0;
    SetupI2CTransmit( 1, 2 );
}

