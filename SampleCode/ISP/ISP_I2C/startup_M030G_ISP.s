;/**************************************************************************//**
; * @file     startup_m030G.s
; * @version  V2.00
; * $Revision: 1 $
; * $Date: 20/06/08 11:57a $
; * @brief    M030G Series Startup Source File
; *
; * @note
; * SPDX-License-Identifier: Apache-2.0  
; * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
; ******************************************************************************/
    IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000200
    ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
    IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000000
    ENDIF
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                                                  ; maximum of 32 External Interrupts are possible
                ;DCD     BOD_IRQHandler
                ;DCD     WDT_IRQHandler
                ;DCD     EINT024_IRQHandler
                ;DCD     EINT135_IRQHandler
                ;DCD     GPAB_IRQHandler
                ;DCD     GPCF_IRQHandler
                ;DCD     Default_Handler
                ;DCD     Default_Handler
                ;DCD     TMR0_IRQHandler
                ;DCD     TMR1_IRQHandler
                ;DCD     Default_Handler
                ;DCD     Default_Handler
                ;DCD     UART0_IRQHandler
                ;DCD     Default_Handler
                ;DCD     SPI0_IRQHandler
                ;DCD     Default_Handler
                ;DCD     Default_Handler
                ;DCD     Default_Handler
                ;DCD     I2C0_IRQHandler
                ;DCD     I2C1_IRQHandler
                ;DCD     Default_Handler
                ;DCD     BPWM_IRQHandler
                ;DCD     Default_Handler
                ;DCD     DAC01_IRQHandler
                ;DCD     DAC23_IRQHandler
                ;DCD     TEMP_IRQHandler
                ;DCD     PDMA_IRQHandler
                ;DCD     Default_Handler
                ;DCD     PWRWU_IRQHandler
                ;DCD     ADC_IRQHandler
                ;DCD     Default_Handler
                ;DCD     Default_Handler



                AREA    |.text|, CODE, READONLY



; Reset Handler

                ENTRY

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main


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

                ; Lock register
                MOVS    R1, #0
                STR     R1, [R0]

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP



; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  EINT024_IRQHandler        [WEAK]
                EXPORT  EINT135_IRQHandler        [WEAK]
                EXPORT  GPAB_IRQHandler           [WEAK]
                EXPORT  GPCF_IRQHandler           [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  BPWM_IRQHandler           [WEAK]
                EXPORT  DAC01_IRQHandler          [WEAK]
                EXPORT  DAC23_IRQHandler          [WEAK]
				EXPORT  TEMP_IRQHandler           [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]

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
                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

                END
