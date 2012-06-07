/*
 * \file   demoSdRw.c
 *
 * \brief  Sample application for HS MMCSD
 *
*/

/* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */

#include "soc_AM335x.h"
#include "hs_mmcsd.h"
#include "edma.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "mmcsd_proto.h"
#include "hs_mmcsdlib.h"
#include "beaglebone.h"
#include "string.h"
#include "demoSdFs.h"

/******************************************************************************
**                      TYPE DEFINITIONS
*******************************************************************************/


/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/* Frequency */
#define HSMMCSD_0_IN_FREQ    96000000 /* 96MHz */
#define HSMMCSD_0_INIT_FREQ  400000   /* 400kHz */

/* EDMA Events */
#define EDMA3_CHA_MMCSD_0_TX        24
#define EDMA3_CHA_MMCSD_0_RX        25

/* EDMA3 Event queue number. */
#define EVT_QUEUE_NUM             (0)

/* EDMA3 Region Number. */
#define REGION_NUMBER             (0)

/* EDMA callback function array */
static void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc, unsigned int status);

/* Block size config */
#define HSMMCSD_BLK_SIZE            512
#define HSMMCSD_RW_BLK                1

/* Global flags for interrupt handling */
volatile unsigned int callbackOccured = 0; 
volatile unsigned int xferCompFlag = 0; 
volatile unsigned int dataTimeout = 0;
volatile unsigned int cmdCompFlag = 0;
volatile unsigned int cmdTimeout = 0; 
volatile unsigned int errFlag = 0;
volatile unsigned int sdBlkSize = HSMMCSD_BLK_SIZE;

/* SD card info structure */
mmcsdCardInfo sdCard;

/* SD Controller info structure */
mmcsdCtrlInfo  ctrlInfo;

/* Global data pointers */
#define HSMMCSD_DATA_SIZE 512

#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment=32
unsigned char data[HSMMCSD_DATA_SIZE];

#elif defined(__TMS470__)
#pragma DATA_ALIGN(data, 32);
unsigned char data[HSMMCSD_DATA_SIZE];
#else
unsigned char data[HSMMCSD_DATA_SIZE] __attribute__ ((aligned (32)))= {0};

#endif

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void HSMMCSDRxDmaConfig(void *ptr, unsigned int blkSize, unsigned int nblks);
static void HSMMCSDTxDmaConfig(void *ptr, unsigned int blkSize, unsigned int blks);
static unsigned int HSMMCSDCmdStatusGet(mmcsdCtrlInfo *ctrl);
static unsigned int HSMMCSDXferStatusGet(mmcsdCtrlInfo *ctrl);
static void HSMMCSDXferSetup(mmcsdCtrlInfo *ctrl, unsigned char rwFlag, void *ptr,
                             unsigned int blkSize, unsigned int nBlks);
static void callback(unsigned int tccNum, unsigned int status);
static void Edma3CompletionIsr(void);
static void Edma3CCErrorIsr(void);
static void HSMMCSDIsr(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/*
 * Check command status
 */
static unsigned int HSMMCSDCmdStatusGet(mmcsdCtrlInfo *ctrl)
{
    unsigned int status = 0;

    while ((cmdCompFlag == 0) && (cmdTimeout == 0));
    
    if (cmdCompFlag)
    {
        status = 1;
        cmdCompFlag = 0;
    }

    if (cmdTimeout)
    {
        status = 0;
        cmdTimeout = 0;
    }

    return status;
}

static unsigned int HSMMCSDXferStatusGet(mmcsdCtrlInfo *ctrl)
{
    unsigned int status = 0;

    while ((xferCompFlag == 0) && (dataTimeout == 0));
    
    if (xferCompFlag)
    {
        status = 1;
        xferCompFlag = 0;
    }

    if (dataTimeout)
    {
        status = 0;
        dataTimeout = 0;
    }

    /* Also, poll for the callback */
    if (HWREG(ctrl->memBase + MMCHS_CMD) & MMCHS_CMD_DP)
    {
        while(callbackOccured == 0);
        callbackOccured = 0;
    }

    ctrlInfo.dmaEnable = 0;

    return status;
}

static void HSMMCSDRxDmaConfig(void *ptr, unsigned int blkSize, unsigned int nblks)
{
    EDMA3CCPaRAMEntry paramSet;

    paramSet.srcAddr    = ctrlInfo.memBase + MMCHS_DATA;
    paramSet.destAddr   = (unsigned int)ptr;
    paramSet.srcBIdx    = 0;
    paramSet.srcCIdx    = 0;
    paramSet.destBIdx   = 4;
    paramSet.destCIdx   = blkSize;
    paramSet.aCnt       = 0x4;
    paramSet.bCnt       = blkSize/4;              
    paramSet.cCnt       = nblks;
    paramSet.bCntReload = 0x0;
    paramSet.linkAddr   = 0xffff;
    paramSet.opt        = 0;

    /* Set OPT */
    paramSet.opt |= ((EDMA3_CHA_MMCSD_0_RX << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* 1. Transmission complition interrupt enable */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* 2. Read FIFO : SRC Constant addr mode */
    paramSet.opt |= (1 << 0);

    /* 3. SRC FIFO width is 32 bit */
    paramSet.opt |= (2 << 8);

    /* 4.  AB-Sync mode */
    paramSet.opt |= (1 << 2);

    /* configure PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MMCSD_0_RX, &paramSet);

    /* Enable the transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MMCSD_0_RX, EDMA3_TRIG_MODE_EVENT);
}

static void HSMMCSDTxDmaConfig(void *ptr, unsigned int blkSize, unsigned int blks)
{
    EDMA3CCPaRAMEntry paramSet;

    paramSet.srcAddr    = (unsigned int)ptr;
    paramSet.destAddr   = ctrlInfo.memBase + MMCHS_DATA;
    paramSet.srcBIdx    = 4;
    paramSet.srcCIdx    = blkSize;
    paramSet.destBIdx   = 0;
    paramSet.destCIdx   = 0;
    paramSet.aCnt       = 0x4;
    paramSet.bCnt       = blkSize/4;
    paramSet.cCnt       = blks;
    paramSet.bCntReload = 0x0;
    paramSet.linkAddr   = 0xffff;
    paramSet.opt        = 0;

    /* Set OPT */
    paramSet.opt |= ((EDMA3_CHA_MMCSD_0_TX << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* 1. Transmission complition interrupt enable */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* 2. Read FIFO : DST Constant addr mode */
    paramSet.opt |= (1 << 1);

    /* 3. DST FIFO width is 32 bit */
    paramSet.opt |= (2 << 8);

    /* 4.  AB-Sync mode */
    paramSet.opt |= (1 << 2);

    /* configure PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MMCSD_0_TX, &paramSet);

    /* Enable the transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MMCSD_0_TX, EDMA3_TRIG_MODE_EVENT);
}

static void HSMMCSDXferSetup(mmcsdCtrlInfo *ctrl, unsigned char rwFlag, void *ptr,
                             unsigned int blkSize, unsigned int nBlks)
{
    if (rwFlag == 1)
    {
        HSMMCSDRxDmaConfig(ptr, blkSize, nBlks);
    }
    else
    {
        HSMMCSDTxDmaConfig(ptr, blkSize, nBlks);
    }

    ctrl->dmaEnable = 1;
    HSMMCSDBlkLenSet(ctrl->memBase, blkSize);

    xferCompFlag = 0;
    callbackOccured = 0;
}


/*
** This function is used as a callback from EDMA3 Completion Handler.
*/
static void callback(unsigned int tccNum, unsigned int status)
{
    callbackOccured = 1;
    EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS, tccNum, EDMA3_TRIG_MODE_EVENT);
}

static void Edma3CompletionIsr(void)
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int isIPR = 0;

    unsigned int indexl;
    unsigned int Cnt = 0;

    indexl = 1;

    isIPR = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_IPR(REGION_NUMBER));
    if(isIPR)
    {
        while ((Cnt < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (indexl != 0u))
        {
            indexl = 0u;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_IPR(REGION_NUMBER));
            while (pendingIrqs)
            {
                if((pendingIrqs & 1u) == TRUE)
                {
                    /**
                    * If the user has not given any callback function
                    * while requesting the TCC, its TCC specific bit
                    * in the IPR register will NOT be cleared.
                    */
                    /* here write to ICR to clear the corresponding IPR bits */
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_ICR(REGION_NUMBER)) = (1u << indexl);

                    if (cb_Fxn[indexl] != NULL)
                    {
                        (*cb_Fxn[indexl])(indexl, EDMA3_XFER_COMPLETE);
                    }
                }
                ++indexl;
                pendingIrqs >>= 1u;
            }
            Cnt++;
        }
    }
}

static void Edma3CCErrorIsr(void)
{
    volatile unsigned int pendingIrqs = 0;
    unsigned int evtqueNum = 0;
    unsigned int index = 1;
    unsigned int Cnt = 0;

    if((HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EMR) != 0 ) || \
       (HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_QEMR) != 0) || \
       (HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERR) != 0))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time, breaks
           when no pending interrupt is found */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EMR);
            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Write to EMCR to clear the corresponding EMR bits.*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EMCR) = (1u<<index);
                    /*Clear any SER*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_SECR(REGION_NUMBER)) = (1u << index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_QEMR);
            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Here write to QEMCR to clear the corresponding QEMR bits*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_QEMCR) = (1u<<index);
                    /*Clear any QSER*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_QSECR(0)) = (1u<<index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERR);
            if (pendingIrqs != 0u)
            {
                /* Process all the pending CC error interrupts. */
                  /* Queue threshold error for different event queues.*/
                for (evtqueNum = 0u; evtqueNum < EDMA3_0_NUM_EVTQUE; evtqueNum++)
                {
                    if((pendingIrqs & (1u << evtqueNum)) != 0u)
                    {
                        /* Clear the error interrupt. */
                        HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERRCLR) = (1u << evtqueNum);
                    }
                 }

                 /* Transfer completion code error. */
                 if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
                 {
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERRCLR) = \
                    (0x01u << EDMA3CC_CCERR_TCCERR_SHIFT);
                 }
                 ++index;
            }

            Cnt++;
        }
    }
}

static void HSMMCSDIsr(void)
{
    volatile unsigned int status = 0;

    status = HSMMCSDIntrStatusGet(ctrlInfo.memBase, 0xFFFFFFFF);
    
    HSMMCSDIntrStatusClear(ctrlInfo.memBase, status);

    if (status & HS_MMCSD_STAT_CMDCOMP)
    {
        cmdCompFlag = 1;
    }

    if (status & HS_MMCSD_STAT_ERR)
    {
        errFlag = status & 0xFFFF0000;

        if (status & HS_MMCSD_STAT_CMDTIMEOUT)
        {
            cmdTimeout = 1;
        }

        if (status & HS_MMCSD_STAT_DATATIMEOUT)
        {
            dataTimeout = 1;
        }
    }

    if (status & HS_MMCSD_STAT_TRNFCOMP)
    {
        xferCompFlag = 1;
    }
}

void HSMMCSDIntRegister(void)
{
    /* Registering HSMMC 0 Interrupt handler */
    IntRegister(SYS_INT_MMCSD0INT, HSMMCSDIsr);

    /* Registering EDMA3 Channel Controller 0 transfer completion interrupt.  */
    IntRegister(SYS_INT_EDMACOMPINT, Edma3CompletionIsr);

    /* Registering EDMA3 Channel Controller 0 Error Interrupt. */
    IntRegister(SYS_INT_EDMAERRINT, Edma3CCErrorIsr);
}

static void HSMMCSDEdmaInit(void)
{
    /* Initializing the EDMA. */
    EDMA3Init(SOC_EDMA30CC_0_REGS, EVT_QUEUE_NUM);

    /* Request DMA Channel and TCC for MMCSD Transmit*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_CHA_MMCSD_0_TX, EDMA3_CHA_MMCSD_0_TX,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for TX*/
    cb_Fxn[EDMA3_CHA_MMCSD_0_TX] = &callback;

    /* Request DMA Channel and TCC for UART Receive */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_CHA_MMCSD_0_RX, EDMA3_CHA_MMCSD_0_RX,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for RX*/
    cb_Fxn[EDMA3_CHA_MMCSD_0_RX] = &callback;
}


/*
** Initialize the MMCSD controller structure for use
*/
static void HSMMCSDControllerSetup(void)
{
    ctrlInfo.memBase = SOC_MMCHS_0_REGS;
    ctrlInfo.ctrlInit = HSMMCSDControllerInit;
    ctrlInfo.xferSetup = HSMMCSDXferSetup;
    ctrlInfo.cmdStatusGet = HSMMCSDCmdStatusGet;
    ctrlInfo.xferStatusGet = HSMMCSDXferStatusGet;
    ctrlInfo.cardPresent = HSMMCSDCardPresent;
    ctrlInfo.cmdSend = HSMMCSDCmdSend;
    ctrlInfo.busWidthConfig = HSMMCSDBusWidthConfig;
    ctrlInfo.busFreqConfig = HSMMCSDBusFreqConfig;
    ctrlInfo.intrMask = (HS_MMCSD_INTR_CMDCOMP | HS_MMCSD_INTR_CMDTIMEOUT |
                            HS_MMCSD_INTR_DATATIMEOUT | HS_MMCSD_INTR_TRNFCOMP);
    ctrlInfo.intrEnable = HSMMCSDIntEnable;
    ctrlInfo.busWidth = (SD_BUS_WIDTH_1BIT | SD_BUS_WIDTH_4BIT);
    ctrlInfo.highspeed = 1;
    ctrlInfo.ocr = (SD_OCR_VDD_3P0_3P1 | SD_OCR_VDD_3P1_3P2);
    ctrlInfo.card = &sdCard;
    ctrlInfo.ipClk = HSMMCSD_0_IN_FREQ;
    ctrlInfo.opClk = HSMMCSD_0_INIT_FREQ;
    sdCard.ctrl = &ctrlInfo;

    callbackOccured = 0;
    xferCompFlag = 0;
    dataTimeout = 0;
    cmdCompFlag = 0;
    cmdTimeout = 0;
}


void HSMMCSDContolInit(void)
{
    HSMMCSDEdmaInit();

    /* Basic controller initializations */
    HSMMCSDControllerSetup();

}

unsigned int HSMMCSDCardPresentStat(void)
{
   if (MMCSDCardPresent(&ctrlInfo) == 0)
   {
       return FALSE;
   }
  
   else 
   {
       return TRUE;
   }
}

void HSMMCSDCardAccessSetup(void)
{
    static unsigned int initStat = 0;

    if(0 == initStat)
    {
        /* Initialize the MMCSD controller */
        MMCSDCtrlInit(&ctrlInfo);
        MMCSDIntEnable(&ctrlInfo);
        HSMMCSDFsMount(0, &sdCard);
        initStat++;
    }
   
    //HSMMCSDFsProcessCmdLine();
}

