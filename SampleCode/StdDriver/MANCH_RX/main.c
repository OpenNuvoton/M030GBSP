/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/10 2:23p $
 * @brief    Use MANCH to do RX test by DMA
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Constant                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
/* MANCH frequency */
#define MANCH_FREQ_IN_HZ              1000

/* Select PDMA channel for MANCH */
#define MANCH_TX_PDMA_CH        0x00
#define MANCH_RX_PDMA_CH        0x01

/* MANCH format */
#define MODE1_FRAME_BYTECOUNT   0x1E
#define MODE1_IDLE              0x00
#define MODE1_PREAMBLE          0x40
#define MODE1_PREAMBLE_NUM      0x05
#define MODE1_EOF               0x7F

#define MODE2_FRAME_BYTECOUNT   0x40
#define MODE2_IDLE              0x00
#define MODE2_PREAMBLE          0x7E
#define MODE2_PREAMBLE_NUM      0x04
#define MODE2_EOF               0x7E

#define MANCH_THOMAS            0x00
#define MANCH_IEEE8023          0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_txBuf[256];
uint8_t g_rxBuf[256];

/*---------------------------------------------------------------------------------------------------------*/
/* Function declaration                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    PDMA_T *pdma;
    MANCH_T *manch;

    uint32_t u32ModeSelect;
    uint32_t u32FrameByteCount;
    uint32_t u32Idle;
    uint32_t u32Preamble;
    uint32_t u32PreambleNum;
//    uint32_t u32FrameEnd;
    uint32_t u32EncodedType;
    uint32_t ii;
    uint8_t u8Option;
    uint8_t *pu8rx;

    /* Set PDMA and MANCH modules */
    pdma = PDMA;
    manch = MANCH;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          MANCH Driver Sample Code                      |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use RX pin (PB.6) to receive MANCH code. \n");
    printf("  To test this sample code, TX signal must be sent from another target board. \n");
    printf("  Press any key when to be ready.\n\n");
    getchar();

    while (1)
    {
        /*--------------------------------------------------------------------------------------*/
        /* MANCH mode selection                                                                 */
        /*--------------------------------------------------------------------------------------*/
        /*==================================================
            MANCH MODE-1
          --------------------------------------------------
              byte count per frame: 0x1E
              IDLE pattern: 0x00
              PREAMBLE pattern: 0x40
              PREAMBLE pattern: 0x40
          --------------------------------------------------
            MANCH MODE-2
          --------------------------------------------------
              byte count per frame: 0x40
              IDLE pattern: 0x00
              PREAMBLE pattern: 0x7E
          ==================================================*/
        printf("  Please select MANCH mode:\n");
        printf("    1: MANCH MODE-1 \n");
        printf("    2: MANCH MODE-2 \n");
        printf("    Other: Stop testing \n\n");
        u8Option = getchar();
        switch(u8Option)
        {
        case '1':
            printf("  MANCH MODE-1 selected \n\n");
            u32ModeSelect = MANCH_MODE1;
            u32FrameByteCount = MODE1_FRAME_BYTECOUNT;
            u32Idle = MODE1_IDLE;
            u32Preamble = MODE1_PREAMBLE;
            u32PreambleNum = MODE1_PREAMBLE_NUM;
//            u32FrameEnd = MODE1_EOF;
            break;
        case '2':
            printf("  MANCH MODE-2 selected \n\n");
            u32ModeSelect = MANCH_MODE2;
            u32FrameByteCount = MODE2_FRAME_BYTECOUNT;
            u32Idle = MODE2_IDLE;
            u32Preamble = MODE2_PREAMBLE;
            u32PreambleNum = MODE2_PREAMBLE_NUM;
//            u32FrameEnd = MODE2_EOF;
            break;
        default:
            u32ModeSelect = MANCH_NONE;
            break;
        }

        if (u32ModeSelect == MANCH_NONE)
        {
            printf("MANCH: Stop testing \n");
            break;
        }

        printf("  Please select encoded type:\n");
        printf("    1: Thomas format \n");
        printf("    2: IEEE802.3 format \n");
        printf("    Other: Thomas format (default) \n\n");
        u8Option = getchar();
        switch(u8Option)
        {
        case '1':
        default:
            printf("  Thomas format selected \n\n");
            u32EncodedType = MANCH_THOMAS;
            break;
        case '2':
            printf("  IEEE802.3 format selected \n\n");
            u32EncodedType = MANCH_IEEE8023;
            break;
        }

        /*--------------------------------------------------------------------------------------*/
        /* Clear RX buffer                                                                      */
        /*--------------------------------------------------------------------------------------*/
        /* Clear RX received data */
        for (ii=0; ii<u32FrameByteCount; ii++)
            g_rxBuf[ii] = 0;

        /*--------------------------------------------------------------------------------------*/
        /* Set MANCH TX and RX related format                                                   */
        /*--------------------------------------------------------------------------------------*/
        /* Open MANCH */
        MANCH_Open(manch, MANCH_FREQ_IN_HZ);

        /* Set MANCH to NONE mode before to set related format */
        MANCH_SetMode(manch, MANCH_NONE);

        /* Set encoded format */
        if (u32EncodedType == MANCH_THOMAS)
            MANCH_ENCODE_THOMAS(manch);
        else
            MANCH_ENCODE_IEEE8023(manch);

        /* Set Frame number */
        MANCH_SetFrameNum(manch, u32FrameByteCount);

        /* Set Idle pattern */
        MANCH_SetIdle(manch, u32Idle);

        /* Set Preamble pattern */
        MANCH_SetPreamble(manch, u32Preamble);

        /* Set Preamble number */
        MANCH_SetPreambleNum(manch, u32PreambleNum);

        /* Set RX polarity */
        MANCH_RX_NOT_INVERTED(manch);

        /* Reset receive FIFO control */
        MANCH_CLR_RX_FIFO(manch);
        MANCH_NOTCLR_RX_FIFO(manch);

        /* Clear Status register */
        MANCH_CLR_STATUS(manch);

        /*--------------------------------------------------------------------------------------*/
        /* Set MANCH mode switch finally                                                        */
        /*--------------------------------------------------------------------------------------*/
        /* Disable RX DMA */
        MANCH_DISABLE_RX_DMA(manch);

        /* Set selected MANCH mode */
        MANCH_SetMode(manch, u32ModeSelect);

        /*--------------------------------------------------------------------------------------*/
        /* Setup PDMA for MANCH RX                                                              */
        /*--------------------------------------------------------------------------------------*/
        /* Reset PDMA module */
        SYS_ResetModule(PDMA_RST);

        /* Enable PDMA channels */
        PDMA_Open(pdma, 1<<MANCH_RX_PDMA_CH);

        /*=======================================================================
            MANCH RX PDMA channel configuration:
          -----------------------------------------------------------------------
              Word length = 8 bits
              Transfer Count = u32FrameByteCount
              Source = MANCH->RXDAT
              Source Address = Fixed
              Destination = g_rxBuf
              Destination Address = Increased
              Burst Type = Single Transfer
         ========================================================================*/
        /* Set transfer width (8 bits) and transfer count */
        PDMA_SetTransferCnt(pdma, MANCH_RX_PDMA_CH, PDMA_WIDTH_8, u32FrameByteCount);
        /* Set source/destination address and attributes */
        PDMA_SetTransferAddr(pdma, MANCH_RX_PDMA_CH, (uint32_t)&manch->RXDAT, PDMA_SAR_FIX, (uint32_t)g_rxBuf, PDMA_DAR_INC);
        /* Set request source; set basic mode. */
        PDMA_SetTransferMode(pdma, MANCH_RX_PDMA_CH, PDMA_MANCH_RX, FALSE, 0);
        /* Single request type; PDMA single request type. */
        PDMA_SetBurstType(pdma, MANCH_RX_PDMA_CH, PDMA_REQ_SINGLE, 0);
        /* Disable table interrupt */
        PDMA->DSCT[MANCH_RX_PDMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

        /*--------------------------------------------------------------------------------------*/
        /* Begin MANCH TX transfer                                                              */
        /*--------------------------------------------------------------------------------------*/
        /* Enable RX DMA */
        MANCH_ENABLE_RX_DMA(manch);

        /*--------------------------------------------------------------------------------------*/
        /* Wait DMA finished                                                                    */
        /*--------------------------------------------------------------------------------------*/
        /* Wait RX PDMA finished */
        while((PDMA->TDSTS & (1<<MANCH_RX_PDMA_CH))==0);

        /* Clear RX PDMA finished flag */
        PDMA_CLR_TD_FLAG(PDMA, (1<<MANCH_RX_PDMA_CH));

        /* Check and clear RX_DONE finished flag */
        while(!MANCH_IS_RXFRAME_DONE(manch));
        MANCH_CLR_RXFRAME_DONE(manch);

        /* Check received data */
        pu8rx = g_rxBuf;

        for (ii=0; ii<u32FrameByteCount; ii++)
        {
            printf("RX received data = 0x%x \n", *pu8rx);
            pu8rx++;
        }

        /* Prepare to do another test */
        printf("*** Prepare to do another RX test ***\n\n");
    }
    while(1);
}

void SYS_Init(void)
{
    /*--------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                    */
    /*--------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable MANCH module clock */
    CLK_EnableModuleClock(MANCH_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*--------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                              */
    /*--------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for MANCH RXD=PB.6 and TXD=PB.7 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk)) |
                    (SYS_GPB_MFPL_PB7MFP_MANCH_TXD | SYS_GPB_MFPL_PB6MFP_MANCH_RXD);

}

void UART0_Init()
{
    /*--------------------------------------------------------------------------------------*/
    /* Init UART                                                                            */
    /*--------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
