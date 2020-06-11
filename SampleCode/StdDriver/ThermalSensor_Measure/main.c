/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/11 7:12p $
 * @brief    Measure the current temperture by thermal sensor.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t volatile done;
uint32_t volatile u32thermalData;

/* Function prototype declaration */
void SYS_Init(void);
void TS_Init(void);

void TEMP_IRQHandler(void)
{
    if (TS_GET_INT_FLAG())
    {
        done = 1;

        /* get tsensor data */
        u32thermalData = TS_GET_DATA();

        /* clear complete flag*/
        TS_CLR_COMPLETE_FLAG();
        while(TS_GET_COMPLETE_FLAG());

        /* clear interrupt flag*/
        TS_CLR_INT_FLAG();
        while(TS_GET_INT_FLAG());
    }
}


/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint8_t  u8Option;
    uint32_t  u32digiPart, u32deciPart;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init TS                                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Tsensor */
    TS_ENABLE();

    /* Clear TSEOC bit */
    TS_CLR_COMPLETE_FLAG();

    /* Disable TS interrupt */
    NVIC_DisableIRQ(TEMP_IRQn);

    /* Clear interrupr flag */
    TS_CLR_INT_FLAG();

    /* Enable TS interrupt */
    NVIC_EnableIRQ(TEMP_IRQn);
    TS_ENABLE_INT();

    while (1)
    {
        printf("\nSelect whether to measure temperature or not: \n");
        printf("[1] Measure temperature  \n");
        printf("[Other] Exit \n");
        u8Option = getchar();
        printf("Select : %d \n", u8Option - 48);

        if (u8Option != '1')
        {
            printf("Exit\n");
            break;
        }
        done = 0;

        /* Enable TSensor tranform */
        TS_TRIGGER();

        /* Wait to tranform completely */
        while(!done);

        /* Read thermal data */
        if (u32thermalData & 0x800)
        {
            /* negative tempature (2's complement)*/
            u32thermalData = ~u32thermalData;
            u32thermalData++;
            u32thermalData &= 0x7FF;
            u32thermalData *= TS_DEGREE_PER_BIT;
            u32digiPart = u32thermalData / 10000;
            u32deciPart = u32thermalData % 10000;
            printf("\nMeasured sensor data = -%d.%d !!!\n\n", u32digiPart, u32deciPart);
        }
        else
        {
            /* positive tempature*/
            u32thermalData &= 0x7FF;
            u32thermalData *= TS_DEGREE_PER_BIT;
            u32digiPart = u32thermalData / 10000;
            u32deciPart = u32thermalData % 10000;
            printf("\nMeasured sensor data = %d.%d !!!\n\n", u32digiPart, u32deciPart);
        }
    }

    /* Close TS */
    NVIC_DisableIRQ(TEMP_IRQn);
    TS_DISABLE();
    TS_DISABLE_INT();

    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
