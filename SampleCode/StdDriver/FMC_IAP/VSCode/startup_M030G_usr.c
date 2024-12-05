/**************************************************************************//**
 * @file     startup_M031Series.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M031Series
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
__NO_RETURN void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__((weak));

/* System Exception Handlers */
void SVC_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__((weak, alias("Default_Handler")));

/* External Interrupt Handlers */
void BOD_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 0: Brown Out detection
void WDT_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 1: Watchdog Timer
void EINT024_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));    // 2: External Input 0,2,4
void EINT135_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));    // 3: External Input 1,3,5
void GPAB_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 4: GPIO Port A, B
void GPCF_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 5: GPIO Port C, F
void TMR4_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 6: Timer 4
void TMR5_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 7: Timer 5
void TMR0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 8: Timer 0
void TMR1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 9: Timer 1
void TMR2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 10: Timer 2
void TMR3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 11: Timer 3
void UART0_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 12: UART0
void SPI0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 14: SPI0
void MANCH_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 17: Manchester encoding
void I2C0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 18: I2C0
void I2C1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 19: I2C1
void BPWM_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 21: BPWM
void DAC01_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 23: DAC0 and DAC1
void DAC23_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 24: DAC2 and DAC3
void TEMP_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 25: Temperature Sensor
void PDMA_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));    // 26: Peripheral DMA
void PWRWU_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 28: Power Wake-up
void ADC_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 29: ADC

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
    /* Interrupts */
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /* Initial Stack Pointer */
    Reset_Handler,                            /* Reset Handler */
    NMI_Handler,                              /* -14 NMI Handler */
    HardFault_Handler,                        /* -13 Hard Fault Handler */
    0,                                        /* -12 Reserved */
    0,                                        /* -11 Reserved */
    0,                                        /* -10 Reserved */
    0,                                        /* -9 Reserved */
    0,                                        /* -8 Reserved */
    0,                                        /* -7 Reserved */
    0,                                        /* -6 Reserved */
    SVC_Handler,                              /* -5 SVC Handler */
    0,                                        /* -4 Reserved */
    0,                                        /* -3 Reserved */
    PendSV_Handler,                           /* -2 PendSV Handler */
    SysTick_Handler,                          /* -1 SysTick Handler */
    
    /* Interrupts */
    BOD_IRQHandler,                           /* 0: Brown Out detection */
    WDT_IRQHandler,                           /* 1: Watchdog Timer */
    EINT024_IRQHandler,                       /* 2: External Input 0, 2, 4 */
    EINT135_IRQHandler,                       /* 3: External Input 1, 3, 5 */
    GPAB_IRQHandler,                          /* 4: GPIO Port A, B */
    GPCF_IRQHandler,                          /* 5: GPIO Port C, F */
    TMR4_IRQHandler,                          /* 6: Timer 4 */
    TMR5_IRQHandler,                          /* 7: Timer 5 */
    TMR0_IRQHandler,                          /* 8: Timer 0 */
    TMR1_IRQHandler,                          /* 9: Timer 1 */
    TMR2_IRQHandler,                          /* 10: Timer 2 */
    TMR3_IRQHandler,                          /* 11: Timer 3 */
    UART0_IRQHandler,                         /* 12: UART0 */
    Default_Handler,                          /* 13: Reserved */
    SPI0_IRQHandler,                          /* 14: SPI0 */
    Default_Handler,                          /* 15: Reserved */
    Default_Handler,                          /* 16: Reserved */
    MANCH_IRQHandler,                         /* 17: Manchester */
    I2C0_IRQHandler,                          /* 18: I2C0 */
    I2C1_IRQHandler,                          /* 19: I2C1 */
    Default_Handler,                          /* 20: Reserved */
    BPWM_IRQHandler,                          /* 21: BPWM */
    Default_Handler,                          /* 22: Reserved */
    DAC01_IRQHandler,                         /* 23: DAC0, DAC1 */
    DAC23_IRQHandler,                         /* 24: DAC2, DAC3 */
    TEMP_IRQHandler,                          /* 25: Temperature Sensor */
    PDMA_IRQHandler,                          /* 26: Peripheral DMA */
    Default_Handler,                          /* 27: Reserved */
    PWRWU_IRQHandler,                         /* 28: Power Wake-up */
    ADC_IRQHandler,                           /* 29: ADC */
    Default_Handler,                          /* 30: Reserved */
    Default_Handler                           /* 31: Reserved */	
};

#if defined ( __GNUC__ )
    #pragma GCC diagnostic pop
#endif

__WEAK void Reset_Handler_PreInit(void)
{
    // Empty function
}

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    __set_PSP((uint32_t)(&__INITIAL_SP));
    __set_MSP((uint32_t)(&__STACK_LIMIT));
    __set_PSP((uint32_t)(&__STACK_LIMIT));

    Reset_Handler_PreInit();
    /* Unlock protected registers */
    SYS_UnlockReg();

    SystemInit();               /* CMSIS System Initialization */

    /* Init POR */
    SYS->PORCTL = 0x5AA5;

    /* Lock protected registers */
    SYS_LockReg();

    __PROGRAM_START();          /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
__WEAK void HardFault_Handler(void)
{
/*    __ASM(
        "MOV     R0, LR  \n"
        "MRS     R1, MSP \n"
        "MRS     R2, PSP \n"
        "LDR     R3, =ProcessHardFault \n"
        "BLX     R3 \n"
        "BX      R0 \n"
    );
*/
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif
