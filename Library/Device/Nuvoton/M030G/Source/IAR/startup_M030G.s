;/**************************************************************************//**
; * @file     startup_M030G.s
; * @version  V3.00
; * $Revision: 2 $
; * $Date: 20/06/08 3:53p $
; * @brief    M030G Series Startup Source File for IAR Platform
; *
; * @note
; * SPDX-License-Identifier: Apache-2.0  
; * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
; *
; ******************************************************************************/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3) ;; 8 bytes alignment

    SECTION .intvec:CODE:NOROOT(2);; 4 bytes alignment

    EXTERN  SystemInit
    EXTERN  __iar_program_start
    PUBLIC  __vector_table

    DATA
__vector_table
    DCD     sfe(CSTACK)
    DCD     Reset_Handler

    DCD     NMI_Handler
    DCD     HardFault_Handler
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     SVC_Handler
    DCD     0
    DCD     0
    DCD     PendSV_Handler
    DCD     SysTick_Handler

    ; External Interrupts
    DCD     BOD_IRQHandler              ; Brownout low voltage detected interrupt
    DCD     WDT_IRQHandler              ; Watch Dog Timer interrupt
    DCD     EINT024_IRQHandler
    DCD     EINT135_IRQHandler
    DCD     GPAB_IRQHandler
    DCD     GPCF_IRQHandler
    DCD     Default_Handler
    DCD     Default_Handler
    DCD     TMR0_IRQHandler             ; Timer 0 interrupt
    DCD     TMR1_IRQHandler             ; Timer 1 interrupt
    DCD     Default_Handler
    DCD     Default_Handler
	DCD     UART0_IRQHandler
	DCD     Default_Handler
	DCD     SPI0_IRQHandler
	DCD     Default_Handler
	DCD     Default_Handler
	DCD     Default_Handler
	DCD     I2C0_IRQHandler
	DCD     I2C1_IRQHandler
	DCD     Default_Handler
	DCD     BPWM_IRQHandler
	DCD     Default_Handler
	DCD     DAC01_IRQHandler
	DCD     DAC23_IRQHandler
	DCD     TEMP_IRQHandler
    DCD     PDMA_IRQHandler
    DCD     Default_Handler
    DCD     PWRWU_IRQHandler
    DCD     ADC_IRQHandler              ; ADC interrupt
    DCD     Default_Handler
    DCD     Default_Handler

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
    THUMB
    PUBWEAK Reset_Handler
    SECTION .text:CODE:REORDER:NOROOT(2)       ; 4 bytes alignment
Reset_Handler
        LDR     R0, =0x40000100
        ; Unlock Register
        LDR     R1, =0x59
        STR     R1, [R0]
        LDR     R1, =0x16
        STR     R1, [R0]
        LDR     R1, =0x88
        STR     R1, [R0]

        ; Init POR
        LDR     R2, =0x40000024
        LDR     R1, =0x00005AA5
        STR     R1, [R2]

        ; Disable NMI (Assign to reserved IRQ)
        LDR     R2, =0x40000380
        LDR     R1, =0x0000001F
        STR     R1, [R2]

        ; Lock register
        MOVS    R1, #0
        STR     R1, [R0]

        LDR      R0, =SystemInit
        BLX      R0
        LDR      R0, =__iar_program_start
        BX       R0

    PUBWEAK HardFault_Handler
    PUBWEAK NMI_Handler
    PUBWEAK SVC_Handler
    PUBWEAK PendSV_Handler
    PUBWEAK SysTick_Handler
    PUBWEAK BOD_IRQHandler
    PUBWEAK WDT_IRQHandler
    PUBWEAK EINT024_IRQHandler
    PUBWEAK EINT135_IRQHandler
    PUBWEAK GPAB_IRQHandler
    PUBWEAK GPCF_IRQHandler
    PUBWEAK TMR0_IRQHandler
    PUBWEAK TMR1_IRQHandler
    PUBWEAK UART0_IRQHandler
    PUBWEAK SPI0_IRQHandler
    PUBWEAK I2C0_IRQHandler
    PUBWEAK I2C1_IRQHandler
    PUBWEAK BPWM_IRQHandler
    PUBWEAK DAC01_IRQHandler
    PUBWEAK DAC23_IRQHandler
    PUBWEAK TEMP_IRQHandler    
    PUBWEAK PDMA_IRQHandler
    PUBWEAK PWRWU_IRQHandler
    PUBWEAK ADC_IRQHandler
    SECTION .text:CODE:REORDER:NOROOT(2)

HardFault_Handler
NMI_Handler
SVC_Handler
PendSV_Handler
SysTick_Handler
BOD_IRQHandler
WDT_IRQHandler
EINT024_IRQHandler
EINT135_IRQHandler
GPAB_IRQHandler
GPCF_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
UART0_IRQHandler
SPI0_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
BPWM_IRQHandler
DAC01_IRQHandler
DAC23_IRQHandler
TEMP_IRQHandler
PDMA_IRQHandler
PWRWU_IRQHandler
ADC_IRQHandler
Default_Handler

    B Default_Handler


    END

