/**************************************************************************//**
 * @file     bpwm_reg.h
 * @version  V1.00
 * @brief    BPWM register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __BPWM_REG_H__
#define __BPWM_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup BPWM Basic Pulse Width Modulation Controller (BPWM)
    Memory Mapped Structure for BPWM Controller
@{ */

typedef struct
{


    /**
     * @var BPWM_T::CTL0
     * Offset: 0x00  BPWM Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CTRLD0    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[1]     |CTRLD1    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[2]     |CTRLD2    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[3]     |CTRLD3    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[4]     |CTRLD4    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[5]     |CTRLD5    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[16]    |IMMLDEN0  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is enabled, WINLDENn and CTRLDn will be invalid.
     * |[17]    |IMMLDEN1  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is enabled, WINLDENn and CTRLDn will be invalid.
     * |[18]    |IMMLDEN2  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is enabled, WINLDENn and CTRLDn will be invalid.
     * |[19]    |IMMLDEN3  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is enabled, WINLDENn and CTRLDn will be invalid.
     * |[20]    |IMMLDEN4  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is enabled, WINLDENn and CTRLDn will be invalid.
     * |[21]    |IMMLDEN5  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is enabled, WINLDENn and CTRLDn will be invalid.
     * |[30]    |DBGHALT   |ICE Debug Mode Counter Halt (Write Protect)
     * |        |          |If counter halt is enabled, BPWM all counters will keep current value until exit ICE debug mode.
     * |        |          |0 = ICE debug mode counter halt disable.
     * |        |          |1 = ICE debug mode counter halt enable.
     * |        |          |Note: This register is write protected. Refer toSYS_REGLCTL register.
     * |[31]    |DBGTRIOFF |ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects BPWM output.
     * |        |          |BPWM pin will be forced as tri-state while ICE debug mode acknowledged.
     * |        |          |1 = ICE debug mode acknowledgement disabled.
     * |        |          |BPWM pin will keep output no matter ICE debug mode acknowledged or not.
     * |        |          |Note: This register is write protected. Refer toSYS_REGLCTL register.
     * @var BPWM_T::CTL1
     * Offset: 0x04  BPWM Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CNTTYPE0  |BPWM Counter Behavior Type 0
     * |        |          |Each bit n controls corresponding BPWM channel n.
     * |        |          |00 = Up counter type (supports in capture mode).
     * |        |          |01 = Down count type (supports in capture mode).
     * |        |          |10 = Up-down counter type.
     * |        |          |11 = Reserved.
     * @var BPWM_T::CLKSRC
     * Offset: 0x10  BPWM Clock Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |ECLKSRC0  |BPWM_CH01 External Clock Source Select
     * |        |          |000 = BPWMx_CLK, x denotes 0 or 1.
     * |        |          |001 = TIMER0 overflow.
     * |        |          |010 = TIMER1 overflow.
     * |        |          |011 = TIMER2 overflow.
     * |        |          |100 = TIMER3 overflow.
     * |        |          |Others = Reserved.
     * @var BPWM_T::CLKPSC
     * Offset: 0x14  BPWM Clock Prescale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |CLKPSC    |BPWM Counter Clock Prescale
     * |        |          |The clock of BPWM counter is decided by clock prescaler
     * |        |          |Each BPWM pair share one BPWM counter clock prescaler
     * |        |          |The clock of BPWM counter is divided by (CLKPSC+ 1)
     * @var BPWM_T::CNTEN
     * Offset: 0x20  BPWM Counter Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |BPWM Counter Enable 0
     * |        |          |0 = BPWM Counter and clock prescaler stop running.
     * |        |          |1 = BPWM Counter and clock prescaler start running.
     * @var BPWM_T::CNTCLR
     * Offset: 0x24  BPWM Clear Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTCLR0   |Clear BPWM Counter Control Bit 0
     * |        |          |It is automatically cleared by hardware.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear 16-bit BPWM counter to 0000H.
     * @var BPWM_T::PERIOD
     * Offset: 0x30  BPWM Period Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |BPWM Period Register
     * |        |          |Up-Count mode: In this mode, BPWM counter counts from 0 to PERIOD, and restarts from 0.
     * |        |          |Down-Count mode: In this mode, BPWM counter counts from PERIOD to 0, and restarts from PERIOD.
     * |        |          |BPWM period time = (PERIOD+1) * BPWM_CLK period.
     * |        |          |Up-Down-Count mode: In this mode, BPWM counter counts from 0 to PERIOD, then decrements to 0 and repeats again.
     * |        |          |BPWM period time = 2 * PERIOD * BPWM_CLK period.
     * @var BPWM_T::CMPDAT[6]
     * Offset: 0x50  BPWM Comparator Register 0~5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMPDAT    |BPWM Comparator Register
     * |        |          |CMPDAT use to compare with CNTR to generate BPWM waveform, interrupt and trigger ADC.
     * |        |          |In independent mode, CMPDAT0~5 denote as 6 independent BPWM_CH0~5 compared point.
     * @var BPWM_T::CNT
     * Offset: 0x90  BPWM Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNT       |BPWM Data Register (Read Only)
     * |        |          |User can monitor CNTR to know the current value in 16-bit period counter.
     * |[16]    |DIRF      |BPWM Direction Indicator Flag (Read Only)
     * |        |          |0 = Counter is Down count.
     * |        |          |1 = Counter is UP count.
     * @var BPWM_T::WGCTL0
     * Offset: 0xB0  BPWM Generation Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |ZPCTL0    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[3:2]   |ZPCTL1    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[5:4]   |ZPCTL2    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[7:6]   |ZPCTL3    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[9:8]   |ZPCTL4    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[11:10] |ZPCTL5    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[17:16] |PRDPCTL0  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[19:18] |PRDPCTL1  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[21:20] |PRDPCTL2  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[23:22] |PRDPCTL3  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[25:24] |PRDPCTL4  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[27:26] |PRDPCTL5  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * @var BPWM_T::WGCTL1
     * Offset: 0xB4  BPWM Generation Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CMPUCTL0  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[3:2]   |CMPUCTL1  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[5:4]   |CMPUCTL2  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[7:6]   |CMPUCTL3  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[9:8]   |CMPUCTL4  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[11:10] |CMPUCTL5  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[17:16] |CMPDCTL0  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[19:18] |CMPDCTL1  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[21:20] |CMPDCTL2  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[23:22] |CMPDCTL3  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[25:24] |CMPDCTL4  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[27:26] |CMPDCTL5  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * @var BPWM_T::MSKEN
     * Offset: 0xB8  BPWM Mask Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKEN0    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[1]     |MSKEN1    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[2]     |MSKEN2    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[3]     |MSKEN3    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[4]     |MSKEN4    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[5]     |MSKEN5    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * @var BPWM_T::MSK
     * Offset: 0xBC  BPWM Mask Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKDAT0   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[1]     |MSKDAT1   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[2]     |MSKDAT2   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[3]     |MSKDAT3   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[4]     |MSKDAT4   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[5]     |MSKDAT5   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * @var BPWM_T::POLCTL
     * Offset: 0xD4  BPWM Pin Polar Inverse Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PINV0     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWM output
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM output polar inverse disabled.
     * |        |          |1 = BPWM output polar inverse enabled.
     * |[1]     |PINV1     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWM output
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM output polar inverse disabled.
     * |        |          |1 = BPWM output polar inverse enabled.
     * |[2]     |PINV2     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWM output
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM output polar inverse disabled.
     * |        |          |1 = BPWM output polar inverse enabled.
     * |[3]     |PINV3     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWM output
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM output polar inverse disabled.
     * |        |          |1 = BPWM output polar inverse enabled.
     * |[4]     |PINV4     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWM output
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM output polar inverse disabled.
     * |        |          |1 = BPWM output polar inverse enabled.
     * |[5]     |PINV5     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWM output
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM output polar inverse disabled.
     * |        |          |1 = BPWM output polar inverse enabled.
     * @var BPWM_T::POEN
     * Offset: 0xD8  BPWM Output Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN0     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM pin at tri-state.
     * |        |          |1 = BPWM pin in output mode.
     * |[1]     |POEN1     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM pin at tri-state.
     * |        |          |1 = BPWM pin in output mode.
     * |[2]     |POEN2     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM pin at tri-state.
     * |        |          |1 = BPWM pin in output mode.
     * |[3]     |POEN3     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM pin at tri-state.
     * |        |          |1 = BPWM pin in output mode.
     * |[4]     |POEN4     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM pin at tri-state.
     * |        |          |1 = BPWM pin in output mode.
     * |[5]     |POEN5     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM pin at tri-state.
     * |        |          |1 = BPWM pin in output mode.
     * @var BPWM_T::INTEN
     * Offset: 0xE0  BPWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZIEN0     |BPWM Zero Point Interrupt Enable 0
     * |        |          |0 = Zero point interrupt disabled.
     * |        |          |1 = Zero point interrupt enabled.
     * |[8]     |PIEN0     |BPWM Period Point Interrupt Enable 0
     * |        |          |0 = Period point interrupt disabled.
     * |        |          |1 = Period point interrupt enabled.
     * |        |          |Note: When up-down counter type period point means center point.
     * |[16]    |CMPUIEN0  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt disabled.
     * |        |          |1 = Compare up count interrupt enabled.
     * |[17]    |CMPUIEN1  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt disabled.
     * |        |          |1 = Compare up count interrupt enabled.
     * |[18]    |CMPUIEN2  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt disabled.
     * |        |          |1 = Compare up count interrupt enabled.
     * |[19]    |CMPUIEN3  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt disabled.
     * |        |          |1 = Compare up count interrupt enabled.
     * |[20]    |CMPUIEN4  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt disabled.
     * |        |          |1 = Compare up count interrupt enabled.
     * |[21]    |CMPUIEN5  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt disabled.
     * |        |          |1 = Compare up count interrupt enabled.
     * |[24]    |CMPDIEN0  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt disabled.
     * |        |          |1 = Compare down count interrupt enabled.
     * |[25]    |CMPDIEN1  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt disabled.
     * |        |          |1 = Compare down count interrupt enabled.
     * |[26]    |CMPDIEN2  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt disabled.
     * |        |          |1 = Compare down count interrupt enabled.
     * |[27]    |CMPDIEN3  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt disabled.
     * |        |          |1 = Compare down count interrupt enabled.
     * |[28]    |CMPDIEN4  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt disabled.
     * |        |          |1 = Compare down count interrupt enabled.
     * |[29]    |CMPDIEN5  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt disabled.
     * |        |          |1 = Compare down count interrupt enabled.
     * @var BPWM_T::INTSTS
     * Offset: 0xE8  BPWM Interrupt Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZIF0      |BPWM Zero Point Interrupt Flag 0
     * |        |          |This bit is set by hardware when BPWM_CH0 counter reaches zero, software can write 1 to clear this bit to zero.
     * |[8]     |PIF0      |BPWM Period Point Interrupt Flag 0
     * |        |          |This bit is set by hardware when BPWM_CH0 counter reaches BPWM_PERIOD0, software can write 1 to clear this bit to zero.
     * |[16]    |CMPUIF0   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[17]    |CMPUIF1   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[18]    |CMPUIF2   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[19]    |CMPUIF3   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[20]    |CMPUIF4   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[21]    |CMPUIF5   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[24]    |CMPDIF0   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[25]    |CMPDIF1   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[26]    |CMPDIF2   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[27]    |CMPDIF3   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[28]    |CMPDIF4   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[29]    |CMPDIF5   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note1: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * @var BPWM_T::DACTRGEN
     * Offset: 0xF4  BPWM Trigger DAC Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZTE       |BPWM Zero Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to zero if this bit is set to1.
     * |        |          |0 = BPWM period point trigger DAC function Disabled.
     * |        |          |1 = BPWM period point trigger DAC function Enabled.
     * |[8]     |PTE       |BPWM Period Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to (PERIODn+1) if this bit is set to1.
     * |        |          |0 = BPWM period point trigger DAC function Disabled.
     * |        |          |1 = BPWM period point trigger DAC function Enabled.
     * |[16]    |CUTRGE0   |BPWM Compare Up Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Up point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Up point trigger DAC function Enabled.
     * |        |          |Note: This bit should keep at 0 when BPWM counter operating in down counter type..
     * |[17]    |CUTRGE1   |BPWM Compare Up Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Up point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Up point trigger DAC function Enabled.
     * |        |          |Note: This bit should keep at 0 when BPWM counter operating in down counter type..
     * |[18]    |CUTRGE2   |BPWM Compare Up Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Up point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Up point trigger DAC function Enabled.
     * |        |          |Note: This bit should keep at 0 when BPWM counter operating in down counter type..
     * |[19]    |CUTRGE3   |BPWM Compare Up Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Up point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Up point trigger DAC function Enabled.
     * |        |          |Note: This bit should keep at 0 when BPWM counter operating in down counter type..
     * |[20]    |CUTRGE4   |BPWM Compare Up Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Up point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Up point trigger DAC function Enabled.
     * |        |          |Note: This bit should keep at 0 when BPWM counter operating in down counter type..
     * |[21]    |CUTRGE5   |BPWM Compare Up Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter counts up to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Up point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Up point trigger DAC function Enabled.
     * |        |          |Note: This bit should keep at 0 when BPWM counter operating in down counter type..
     * |[24]    |CDTRGE0   |BPWM Compare Down Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Down count point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Down count point trigger DAC function Enabled.
     * |        |          |Note1: This bit should keep at 0 when BPWM counter operating in up counter type.
     * |        |          |Note2: In complementary mode, CDTRGEN1, 3, 5 is used as another CDTRGEN for channel 0, 2, 4.
     * |[25]    |CDTRGE1   |BPWM Compare Down Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Down count point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Down count point trigger DAC function Enabled.
     * |        |          |Note1: This bit should keep at 0 when BPWM counter operating in up counter type.
     * |        |          |Note2: In complementary mode, CDTRGEN1, 3, 5 is used as another CDTRGEN for channel 0, 2, 4.
     * |[26]    |CDTRGE2   |BPWM Compare Down Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Down count point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Down count point trigger DAC function Enabled.
     * |        |          |Note1: This bit should keep at 0 when BPWM counter operating in up counter type.
     * |        |          |Note2: In complementary mode, CDTRGEN1, 3, 5 is used as another CDTRGEN for channel 0, 2, 4.
     * |[27]    |CDTRGE3   |BPWM Compare Down Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Down count point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Down count point trigger DAC function Enabled.
     * |        |          |Note1: This bit should keep at 0 when BPWM counter operating in up counter type.
     * |        |          |Note2: In complementary mode, CDTRGEN1, 3, 5 is used as another CDTRGEN for channel 0, 2, 4.
     * |[28]    |CDTRGE4   |BPWM Compare Down Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Down count point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Down count point trigger DAC function Enabled.
     * |        |          |Note1: This bit should keep at 0 when BPWM counter operating in up counter type.
     * |        |          |Note2: In complementary mode, CDTRGEN1, 3, 5 is used as another CDTRGEN for channel 0, 2, 4.
     * |[29]    |CDTRGE5   |BPWM Compare Down Count Point Trigger DAC Enable Bits
     * |        |          |BPWM can trigger DAC to start action when BPWM counter down count to CMP if this bit is set to1.
     * |        |          |0 = BPWM Compare Down count point trigger DAC function Disabled.
     * |        |          |1 = BPWM Compare Down count point trigger DAC function Enabled.
     * |        |          |Note1: This bit should keep at 0 when BPWM counter operating in up counter type.
     * |        |          |Note2: In complementary mode, CDTRGEN1, 3, 5 is used as another CDTRGEN for channel 0, 2, 4.
     * @var BPWM_T::ADCTS0
     * Offset: 0xF8  BPWM Trigger ADC Source Select Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRGSEL0   |BPWM_CH0 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH0 zero point.
     * |        |          |0001 = BPWM_CH0 period point.
     * |        |          |0010 = BPWM_CH0 zero or period point.
     * |        |          |0011 = BPWM_CH0 up-count CMPDAT point.
     * |        |          |0100 = BPWM_CH0 down-count CMPDAT point.
     * |        |          |0101 = Reserved.
     * |        |          |0110 = Reserved.
     * |        |          |0111 = Reserved.
     * |        |          |1000 = BPWM_CH1 up-count CMPDAT point.
     * |        |          |1001 = BPWM_CH1 down-count CMPDAT point.
     * |        |          |Others reserved
     * |[7]     |TRGEN0    |BPWM_CH0 Trigger ADC enable bit
     * |[11:8]  |TRGSEL1   |BPWM_CH1 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH0 zero point.
     * |        |          |0001 = BPWM_CH0 period point.
     * |        |          |0010 = BPWM_CH0 zero or period point.
     * |        |          |0011 = BPWM_CH0 up-count CMPDAT point.
     * |        |          |0100 = BPWM_CH0 down-count CMPDAT point.
     * |        |          |0101 = Reserved.
     * |        |          |0110 = Reserved.
     * |        |          |0111 = Reserved.
     * |        |          |1000 = BPWM_CH1 up-count CMPDAT point.
     * |        |          |1001 = BPWM_CH1 down-count CMPDAT point.
     * |        |          |Others reserved
     * |[15]    |TRGEN1    |BPWM_CH1 Trigger ADC enable bit
     * |[19:16] |TRGSEL2   |BPWM_CH2 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH2 zero point.
     * |        |          |0001 = BPWM_CH2 period point.
     * |        |          |0010 = BPWM_CH2 zero or period point.
     * |        |          |0011 = BPWM_CH2 up-count CMPDAT point.
     * |        |          |0100 = BPWM_CH2 down-count CMPDAT point.
     * |        |          |0101 = Reserved.
     * |        |          |0110 = Reserved.
     * |        |          |0111 = Reserved.
     * |        |          |1000 = BPWM_CH3 up-count CMPDAT point.
     * |        |          |1001 = BPWM_CH3 down-count CMPDAT point.
     * |        |          |Others reserved
     * |[23]    |TRGEN2    |BPWM_CH2 Trigger ADC enable bit
     * |[27:24] |TRGSEL3   |BPWM_CH3 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH2 zero point.
     * |        |          |0001 = BPWM_CH2 period point.
     * |        |          |0010 = BPWM_CH2 zero or period point.
     * |        |          |0011 = BPWM_CH2 up-count CMPDAT point.
     * |        |          |0100 = BPWM_CH2 down-count CMPDAT point.
     * |        |          |0101 = Reserved.
     * |        |          |0110 = Reserved.
     * |        |          |0111 = Reserved.
     * |        |          |1000 = BPWM_CH3 up-count CMPDAT point.
     * |        |          |1001 = BPWM_CH3 down-count CMPDAT point.
     * |        |          |Others reserved.
     * |[31]    |TRGEN3    |BPWM_CH3 Trigger ADC enable bit
     * @var BPWM_T::ADCTS1
     * Offset: 0xFC  BPWM Trigger ADC Source Select Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRGSEL4   |BPWM_CH4 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH4 zero point.
     * |        |          |0001 = BPWM_CH4 period point.
     * |        |          |0010 = BPWM_CH4 zero or period point.
     * |        |          |0011 = BPWM_CH4 up-count CMPDAT point.
     * |        |          |0100 = BPWM_CH4 down-count CMPDAT point.
     * |        |          |0101 = Reserved.
     * |        |          |0110 = Reserved.
     * |        |          |0111 = Reserved.
     * |        |          |1000 = BPWM_CH5 up-count CMPDAT point.
     * |        |          |1001 = BPWM_CH5 down-count CMPDAT point.
     * |        |          |Others reserved
     * |[7]     |TRGEN4    |BPWM_CH4 Trigger ADC enable bit
     * |[11:8]  |TRGSEL5   |BPWM_CH5 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH4 zero point.
     * |        |          |0001 = BPWM_CH4 period point.
     * |        |          |0010 = BPWM_CH4 zero or period point.
     * |        |          |0011 = BPWM_CH4 up-count CMPDAT point.
     * |        |          |0100 = BPWM_CH4 down-count CMPDAT point.
     * |        |          |0101 = Reserved.
     * |        |          |0110 = Reserved.
     * |        |          |0111 = Reserved.
     * |        |          |1000 = BPWM_CH5 up-count CMPDAT point.
     * |        |          |1001 = BPWM_CH5 down-count CMPDAT point.
     * |        |          |Others reserved
     * |[15]    |TRGEN5    |BPWM_CH5 Trigger ADC enable bit
     * @var BPWM_T::STATUS
     * Offset: 0x120  BPWM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTMAX0   |Time-base Counter 0 Equal to 0xFFFF Latched Status
     * |        |          |0 = indicates the time-base counter never reached its maximum value 0xFFFF.
     * |        |          |1 = indicates the time-base counter reached its maximum value, software can write 1 to clear this bit.
     * |[16]    |ADCTRG0  |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Indicates no ADC start of conversion trigger event has occurred.
     * |        |          |1 = Indicates an ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[17]    |ADCTRG1  |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Indicates no ADC start of conversion trigger event has occurred.
     * |        |          |1 = Indicates an ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[18]    |ADCTRG2  |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Indicates no ADC start of conversion trigger event has occurred.
     * |        |          |1 = Indicates an ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[19]    |ADCTRG3  |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Indicates no ADC start of conversion trigger event has occurred.
     * |        |          |1 = Indicates an ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[20]    |ADCTRG4  |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Indicates no ADC start of conversion trigger event has occurred.
     * |        |          |1 = Indicates an ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[21]    |ADCTRG5  |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Indicates no ADC start of conversion trigger event has occurred.
     * |        |          |1 = Indicates an ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * @var BPWM_T::CAPINEN
     * Offset: 0x200  BPWM Capture Input Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPINEN0  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[1]     |CAPINEN1  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[2]     |CAPINEN2  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[3]     |CAPINEN3  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[4]     |CAPINEN4  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[5]     |CAPINEN5  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * @var BPWM_T::CAPCTL
     * Offset: 0x204  BPWM Capture Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPEN0    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function disabled. RCAPDAT/FCAPDAT register will not be updated.
     * |        |          |1 = Capture function enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[1]     |CAPEN1    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function disabled. RCAPDAT/FCAPDAT register will not be updated.
     * |        |          |1 = Capture function enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[2]     |CAPEN2    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function disabled. RCAPDAT/FCAPDAT register will not be updated.
     * |        |          |1 = Capture function enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[3]     |CAPEN3    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function disabled. RCAPDAT/FCAPDAT register will not be updated.
     * |        |          |1 = Capture function enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[4]     |CAPEN4    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function disabled. RCAPDAT/FCAPDAT register will not be updated.
     * |        |          |1 = Capture function enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[5]     |CAPEN5    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function disabled. RCAPDAT/FCAPDAT register will not be updated.
     * |        |          |1 = Capture function enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[8]     |CAPINV0   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter disabled.
     * |        |          |1 = Capture source inverter enabled. Reverse the input signal from GPIO.
     * |[9]     |CAPINV1   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter disabled.
     * |        |          |1 = Capture source inverter enabled. Reverse the input signal from GPIO.
     * |[10]    |CAPINV2   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter disabled.
     * |        |          |1 = Capture source inverter enabled. Reverse the input signal from GPIO.
     * |[11]    |CAPINV3   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter disabled.
     * |        |          |1 = Capture source inverter enabled. Reverse the input signal from GPIO.
     * |[12]    |CAPINV4   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter disabled.
     * |        |          |1 = Capture source inverter enabled. Reverse the input signal from GPIO.
     * |[13]    |CAPINV5   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter disabled.
     * |        |          |1 = Capture source inverter enabled. Reverse the input signal from GPIO.
     * |[16]    |RCRLDEN0  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter disabled.
     * |        |          |1 = Rising capture reload counter enabled.
     * |[17]    |RCRLDEN1  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter disabled.
     * |        |          |1 = Rising capture reload counter enabled.
     * |[18]    |RCRLDEN2  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter disabled.
     * |        |          |1 = Rising capture reload counter enabled.
     * |[19]    |RCRLDEN3  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter disabled.
     * |        |          |1 = Rising capture reload counter enabled.
     * |[20]    |RCRLDEN4  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter disabled.
     * |        |          |1 = Rising capture reload counter enabled.
     * |[21]    |RCRLDEN5  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter disabled.
     * |        |          |1 = Rising capture reload counter enabled.
     * |[24]    |FCRLDEN0  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter disabled.
     * |        |          |1 = Falling capture reload counter enabled.
     * |[25]    |FCRLDEN1  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter disabled.
     * |        |          |1 = Falling capture reload counter enabled.
     * |[26]    |FCRLDEN2  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter disabled.
     * |        |          |1 = Falling capture reload counter enabled.
     * |[27]    |FCRLDEN3  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter disabled.
     * |        |          |1 = Falling capture reload counter enabled.
     * |[28]    |FCRLDEN4  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter disabled.
     * |        |          |1 = Falling capture reload counter enabled.
     * |[29]    |FCRLDEN5  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter disabled.
     * |        |          |1 = Falling capture reload counter enabled.
     * @var BPWM_T::CAPSTS
     * Offset: 0x208  BPWM Capture Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRIFOV0   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[1]     |CRIFOV1   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[2]     |CRIFOV2   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[3]     |CRIFOV3   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[4]     |CRIFOV4   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[5]     |CRIFOV5   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[8]     |CFIFOV0   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[9]     |CFIFOV1   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[10]    |CFIFOV2   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[11]    |CFIFOV3   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[12]    |CFIFOV4   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[13]    |CFIFOV5   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * @var BPWM_T::RCAPDAT0
     * Offset: 0x20C  BPWM Rising Capture Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data Register (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::FCAPDAT0
     * Offset: 0x210  BPWM Falling Capture Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data Register (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::RCAPDAT1
     * Offset: 0x214  BPWM Rising Capture Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data Register (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::FCAPDAT1
     * Offset: 0x218  BPWM Falling Capture Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data Register (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::RCAPDAT2
     * Offset: 0x21C  BPWM Rising Capture Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data Register (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::FCAPDAT2
     * Offset: 0x220  BPWM Falling Capture Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data Register (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::RCAPDAT3
     * Offset: 0x224  BPWM Rising Capture Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data Register (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::FCAPDAT3
     * Offset: 0x228  BPWM Falling Capture Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data Register (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::RCAPDAT4
     * Offset: 0x22C  BPWM Rising Capture Data Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data Register (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::FCAPDAT4
     * Offset: 0x230  BPWM Falling Capture Data Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data Register (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::RCAPDAT5
     * Offset: 0x234  BPWM Rising Capture Data Register 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data Register (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::FCAPDAT5
     * Offset: 0x238  BPWM Falling Capture Data Register 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data Register (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     * @var BPWM_T::CAPIEN
     * Offset: 0x250  BPWM Capture Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CAPRIENn  |BPWM Capture Rising Latch Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture rising edge latch interrupt disabled.
     * |        |          |1 = Capture rising edge latch interrupt enabled.
     * |[13:8]  |CAPFIENn  |BPWM Capture Falling Latch Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture falling edge latch interrupt disabled.
     * |        |          |1 = Capture falling edge latch interrupt enabled.
     * @var BPWM_T::CAPIF
     * Offset: 0x254  BPWM Capture Interrupt Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPRIF0   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[1]     |CAPRIF1   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[2]     |CAPRIF2   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[3]     |CAPRIF3   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[4]     |CAPRIF4   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[5]     |CAPRIF5   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[8]     |CAPFIF0   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[9]     |CAPFIF1   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[10]    |CAPFIF2   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[11]    |CAPFIF3   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[12]    |CAPFIF4   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[13]    |CAPFIF5   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * @var BPWM_T::PBUF
     * Offset: 0x304  BPWM PERIOD Buffer
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PBUF      |BPWM Period Register Buffer (Read Only)
     * |        |          |Used as PERIOD active register.
     * @var BPWM_T::CMPBUF[6]
     * Offset: 0x31C  BPWM CMPDAT 0~5 Buffer
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMPBUF    |BPWM Comparator Buffer (Read Only)
     * |        |          |Used as CMP active register.
     */
    __IO uint32_t CTL0;                  /*!< [0x0000] BPWM Control Register 0                                          */
    __IO uint32_t CTL1;                  /*!< [0x0004] BPWM Control Register 1                                          */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t CLKSRC;                /*!< [0x0010] BPWM Clock Source Register                                       */
    __IO uint32_t CLKPSC;                /*!< [0x0014] BPWM Clock Prescale Register                                     */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t CNTEN;                 /*!< [0x0020] BPWM Counter Enable Register                                     */
    __IO uint32_t CNTCLR;                /*!< [0x0024] BPWM Clear Counter Register                                      */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t PERIOD;                /*!< [0x0030] BPWM Period Register                                             */
    __I  uint32_t RESERVE3[7];
    __IO uint32_t CMPDAT[6];             /*!< [0x0050] BPWM Comparator Register                                         */
    __I  uint32_t RESERVE4[10];
    __I  uint32_t CNT;                   /*!< [0x0090] BPWM Counter Register                                            */
    __I  uint32_t RESERVE5[7];
    __IO uint32_t WGCTL0;                /*!< [0x00b0] BPWM Generation Register 0                                       */
    __IO uint32_t WGCTL1;                /*!< [0x00b4] BPWM Generation Register 1                                       */
    __IO uint32_t MSKEN;                 /*!< [0x00b8] BPWM Mask Enable Register                                        */
    __IO uint32_t MSK;                   /*!< [0x00bc] BPWM Mask Data Register                                          */
    __I  uint32_t RESERVE6[5];
    __IO uint32_t POLCTL;                /*!< [0x00d4] BPWM Pin Polar Inverse Register                                  */
    __IO uint32_t POEN;                  /*!< [0x00d8] BPWM Output Enable Register                                      */
    __I  uint32_t RESERVE7[1];
    __IO uint32_t INTEN;                 /*!< [0x00e0] BPWM Interrupt Enable Register                                   */
    __I  uint32_t RESERVE8[1];
    __IO uint32_t INTSTS;                /*!< [0x00e8] BPWM Interrupt Flag Register                                     */
    __I  uint32_t RESERVE9[2];
    __IO uint32_t DACTRGEN;              /*!< [0x00f4] BPWM Trigger DAC Enable Register                                 */
    __IO uint32_t ADCTS0;                /*!< [0x00f8] BPWM Trigger ADC Source Select Register 0                        */
    __IO uint32_t ADCTS1;                /*!< [0x00fc] BPWM Trigger ADC Source Select Register 1                        */
    __I  uint32_t RESERVE10[8];
    __IO uint32_t STATUS;                /*!< [0x0120] BPWM Status Register                                             */
    __I  uint32_t RESERVE11[55];
    __IO uint32_t CAPINEN;               /*!< [0x0200] BPWM Capture Input Enable Register                               */
    __IO uint32_t CAPCTL;                /*!< [0x0204] BPWM Capture Control Register                                    */
    __I  uint32_t CAPSTS;                /*!< [0x0208] BPWM Capture Status Register                                     */
    __I  uint32_t RCAPDAT0;              /*!< [0x020c] BPWM Rising Capture Data Register 0                              */
    __I  uint32_t FCAPDAT0;              /*!< [0x0210] BPWM Falling Capture Data Register 0                             */
    __I  uint32_t RCAPDAT1;              /*!< [0x0214] BPWM Rising Capture Data Register 1                              */
    __I  uint32_t FCAPDAT1;              /*!< [0x0218] BPWM Falling Capture Data Register 1                             */
    __I  uint32_t RCAPDAT2;              /*!< [0x021c] BPWM Rising Capture Data Register 2                              */
    __I  uint32_t FCAPDAT2;              /*!< [0x0220] BPWM Falling Capture Data Register 2                             */
    __I  uint32_t RCAPDAT3;              /*!< [0x0224] BPWM Rising Capture Data Register 3                              */
    __I  uint32_t FCAPDAT3;              /*!< [0x0228] BPWM Falling Capture Data Register 3                             */
    __I  uint32_t RCAPDAT4;              /*!< [0x022c] BPWM Rising Capture Data Register 4                              */
    __I  uint32_t FCAPDAT4;              /*!< [0x0230] BPWM Falling Capture Data Register 4                             */
    __I  uint32_t RCAPDAT5;              /*!< [0x0234] BPWM Rising Capture Data Register 5                              */
    __I  uint32_t FCAPDAT5;              /*!< [0x0238] BPWM Falling Capture Data Register 5                             */
    __I  uint32_t RESERVE12[5];
    __IO uint32_t CAPIEN;                /*!< [0x0250] BPWM Capture Interrupt Enable Register                           */
    __IO uint32_t CAPIF;                 /*!< [0x0254] BPWM Capture Interrupt Flag Register                             */
    __I  uint32_t RESERVE13[43];
    __I  uint32_t PBUF;                  /*!< [0x0304] BPWM PERIOD Buffer                                               */
    __I  uint32_t RESERVE14[5];
    __I  uint32_t CMPBUF[6];             /*!< [0x031c] BPWM CMPDAT Buffer                                             */

} BPWM_T;

/**
    @addtogroup BPWM_CONST BPWM Bit Field Definition
    Constant Definitions for BPWM Controller
@{ */

#define BPWM_CTL0_CTRLD0_Pos            (0)                                               /*!< BPWM_T::CTL0: CTRLD0 Position         */
#define BPWM_CTL0_CTRLD0_Msk            (0x1ul << BPWM_CTL0_CTRLD0_Pos)                  /*!< BPWM_T::CTL0: CTRLD0 Mask             */

#define BPWM_CTL0_CTRLD1_Pos            (1)                                               /*!< BPWM_T::CTL0: CTRLD1 Position         */
#define BPWM_CTL0_CTRLD1_Msk            (0x1ul << BPWM_CTL0_CTRLD1_Pos)                  /*!< BPWM_T::CTL0: CTRLD1 Mask             */

#define BPWM_CTL0_CTRLD2_Pos            (2)                                               /*!< BPWM_T::CTL0: CTRLD2 Position         */
#define BPWM_CTL0_CTRLD2_Msk            (0x1ul << BPWM_CTL0_CTRLD2_Pos)                  /*!< BPWM_T::CTL0: CTRLD2 Mask             */

#define BPWM_CTL0_CTRLD3_Pos            (3)                                               /*!< BPWM_T::CTL0: CTRLD3 Position         */
#define BPWM_CTL0_CTRLD3_Msk            (0x1ul << BPWM_CTL0_CTRLD3_Pos)                  /*!< BPWM_T::CTL0: CTRLD3 Mask             */

#define BPWM_CTL0_CTRLD4_Pos            (4)                                               /*!< BPWM_T::CTL0: CTRLD4 Position         */
#define BPWM_CTL0_CTRLD4_Msk            (0x1ul << BPWM_CTL0_CTRLD4_Pos)                  /*!< BPWM_T::CTL0: CTRLD4 Mask             */

#define BPWM_CTL0_CTRLD5_Pos            (5)                                               /*!< BPWM_T::CTL0: CTRLD5 Position         */
#define BPWM_CTL0_CTRLD5_Msk            (0x1ul << BPWM_CTL0_CTRLD5_Pos)                  /*!< BPWM_T::CTL0: CTRLD5 Mask             */

#define BPWM_CTL0_IMMLDEN0_Pos          (16)                                              /*!< BPWM_T::CTL0: IMMLDEN0 Position       */
#define BPWM_CTL0_IMMLDEN0_Msk          (0x1ul << BPWM_CTL0_IMMLDEN0_Pos)                /*!< BPWM_T::CTL0: IMMLDEN0 Mask           */

#define BPWM_CTL0_IMMLDEN1_Pos          (17)                                              /*!< BPWM_T::CTL0: IMMLDEN1 Position       */
#define BPWM_CTL0_IMMLDEN1_Msk          (0x1ul << BPWM_CTL0_IMMLDEN1_Pos)                /*!< BPWM_T::CTL0: IMMLDEN1 Mask           */

#define BPWM_CTL0_IMMLDEN2_Pos          (18)                                              /*!< BPWM_T::CTL0: IMMLDEN2 Position       */
#define BPWM_CTL0_IMMLDEN2_Msk          (0x1ul << BPWM_CTL0_IMMLDEN2_Pos)                /*!< BPWM_T::CTL0: IMMLDEN2 Mask           */

#define BPWM_CTL0_IMMLDEN3_Pos          (19)                                              /*!< BPWM_T::CTL0: IMMLDEN3 Position       */
#define BPWM_CTL0_IMMLDEN3_Msk          (0x1ul << BPWM_CTL0_IMMLDEN3_Pos)                /*!< BPWM_T::CTL0: IMMLDEN3 Mask           */

#define BPWM_CTL0_IMMLDEN4_Pos          (20)                                              /*!< BPWM_T::CTL0: IMMLDEN4 Position       */
#define BPWM_CTL0_IMMLDEN4_Msk          (0x1ul << BPWM_CTL0_IMMLDEN4_Pos)                /*!< BPWM_T::CTL0: IMMLDEN4 Mask           */

#define BPWM_CTL0_IMMLDEN5_Pos          (21)                                              /*!< BPWM_T::CTL0: IMMLDEN5 Position       */
#define BPWM_CTL0_IMMLDEN5_Msk          (0x1ul << BPWM_CTL0_IMMLDEN5_Pos)                /*!< BPWM_T::CTL0: IMMLDEN5 Mask           */

#define BPWM_CTL0_DBGHALT_Pos           (30)                                              /*!< BPWM_T::CTL0: DBGHALT Position        */
#define BPWM_CTL0_DBGHALT_Msk           (0x1ul << BPWM_CTL0_DBGHALT_Pos)                 /*!< BPWM_T::CTL0: DBGHALT Mask            */

#define BPWM_CTL0_DBGTRIOFF_Pos         (31)                                              /*!< BPWM_T::CTL0: DBGTRIOFF Position      */
#define BPWM_CTL0_DBGTRIOFF_Msk         (0x1ul << BPWM_CTL0_DBGTRIOFF_Pos)               /*!< BPWM_T::CTL0: DBGTRIOFF Mask          */

#define BPWM_CTL1_CNTTYPE0_Pos          (0)                                               /*!< BPWM_T::CTL1: CNTTYPE0 Position       */
#define BPWM_CTL1_CNTTYPE0_Msk          (0x3ul << BPWM_CTL1_CNTTYPE0_Pos)                /*!< BPWM_T::CTL1: CNTTYPE0 Mask           */

#define BPWM_CLKSRC_ECLKSRC0_Pos        (0)                                               /*!< BPWM_T::CLKSRC: ECLKSRC0 Position     */
#define BPWM_CLKSRC_ECLKSRC0_Msk        (0x7ul << BPWM_CLKSRC_ECLKSRC0_Pos)              /*!< BPWM_T::CLKSRC: ECLKSRC0 Mask         */

#define BPWM_CLKPSC_CLKPSC_Pos          (0)                                               /*!< BPWM_T::CLKPSC: CLKPSC Position       */
#define BPWM_CLKPSC_CLKPSC_Msk          (0xffful << BPWM_CLKPSC_CLKPSC_Pos)              /*!< BPWM_T::CLKPSC: CLKPSC Mask           */

#define BPWM_CNTEN_CNTEN0_Pos           (0)                                               /*!< BPWM_T::CNTEN: CNTEN0 Position        */
#define BPWM_CNTEN_CNTEN0_Msk           (0x1ul << BPWM_CNTEN_CNTEN0_Pos)                 /*!< BPWM_T::CNTEN: CNTEN0 Mask            */

#define BPWM_CNTCLR_CNTCLR0_Pos         (0)                                               /*!< BPWM_T::CNTCLR: CNTCLR0 Position      */
#define BPWM_CNTCLR_CNTCLR0_Msk         (0x1ul << BPWM_CNTCLR_CNTCLR0_Pos)               /*!< BPWM_T::CNTCLR: CNTCLR0 Mask          */

#define BPWM_PERIOD_PERIOD_Pos          (0)                                               /*!< BPWM_T::PERIOD: PERIOD Position       */
#define BPWM_PERIOD_PERIOD_Msk          (0xfffful << BPWM_PERIOD_PERIOD_Pos)             /*!< BPWM_T::PERIOD: PERIOD Mask           */

#define BPWM_CMPDAT_CMPDAT_Pos         (0)                                               /*!< BPWM_T::CMPDAT0: CMPDAT Position      */
#define BPWM_CMPDAT_CMPDAT_Msk         (0xfffful << BPWM_CMPDAT_CMPDAT_Pos)            /*!< BPWM_T::CMPDAT0: CMPDAT Mask          */

#define BPWM_CNT_CNT_Pos                (0)                                               /*!< BPWM_T::CNT: CNT Position             */
#define BPWM_CNT_CNT_Msk                (0xfffful << BPWM_CNT_CNT_Pos)                   /*!< BPWM_T::CNT: CNT Mask                 */

#define BPWM_CNT_DIRF_Pos               (16)                                              /*!< BPWM_T::CNT: DIRF Position            */
#define BPWM_CNT_DIRF_Msk               (0x1ul << BPWM_CNT_DIRF_Pos)                     /*!< BPWM_T::CNT: DIRF Mask                */

#define BPWM_WGCTL0_ZPCTL0_Pos          (0)                                               /*!< BPWM_T::WGCTL0: ZPCTL0 Position       */
#define BPWM_WGCTL0_ZPCTL0_Msk          (0x3ul << BPWM_WGCTL0_ZPCTL0_Pos)                /*!< BPWM_T::WGCTL0: ZPCTL0 Mask           */

#define BPWM_WGCTL0_ZPCTL1_Pos          (2)                                               /*!< BPWM_T::WGCTL0: ZPCTL1 Position       */
#define BPWM_WGCTL0_ZPCTL1_Msk          (0x3ul << BPWM_WGCTL0_ZPCTL1_Pos)                /*!< BPWM_T::WGCTL0: ZPCTL1 Mask           */

#define BPWM_WGCTL0_ZPCTL2_Pos          (4)                                               /*!< BPWM_T::WGCTL0: ZPCTL2 Position       */
#define BPWM_WGCTL0_ZPCTL2_Msk          (0x3ul << BPWM_WGCTL0_ZPCTL2_Pos)                /*!< BPWM_T::WGCTL0: ZPCTL2 Mask           */

#define BPWM_WGCTL0_ZPCTL3_Pos          (6)                                               /*!< BPWM_T::WGCTL0: ZPCTL3 Position       */
#define BPWM_WGCTL0_ZPCTL3_Msk          (0x3ul << BPWM_WGCTL0_ZPCTL3_Pos)                /*!< BPWM_T::WGCTL0: ZPCTL3 Mask           */

#define BPWM_WGCTL0_ZPCTL4_Pos          (8)                                               /*!< BPWM_T::WGCTL0: ZPCTL4 Position       */
#define BPWM_WGCTL0_ZPCTL4_Msk          (0x3ul << BPWM_WGCTL0_ZPCTL4_Pos)                /*!< BPWM_T::WGCTL0: ZPCTL4 Mask           */

#define BPWM_WGCTL0_ZPCTL5_Pos          (10)                                              /*!< BPWM_T::WGCTL0: ZPCTL5 Position       */
#define BPWM_WGCTL0_ZPCTL5_Msk          (0x3ul << BPWM_WGCTL0_ZPCTL5_Pos)                /*!< BPWM_T::WGCTL0: ZPCTL5 Mask           */

#define BPWM_WGCTL0_PRDPCTL0_Pos        (16)                                              /*!< BPWM_T::WGCTL0: PRDPCTL0 Position     */
#define BPWM_WGCTL0_PRDPCTL0_Msk        (0x3ul << BPWM_WGCTL0_PRDPCTL0_Pos)              /*!< BPWM_T::WGCTL0: PRDPCTL0 Mask         */

#define BPWM_WGCTL0_PRDPCTL1_Pos        (18)                                              /*!< BPWM_T::WGCTL0: PRDPCTL1 Position     */
#define BPWM_WGCTL0_PRDPCTL1_Msk        (0x3ul << BPWM_WGCTL0_PRDPCTL1_Pos)              /*!< BPWM_T::WGCTL0: PRDPCTL1 Mask         */

#define BPWM_WGCTL0_PRDPCTL2_Pos        (20)                                              /*!< BPWM_T::WGCTL0: PRDPCTL2 Position     */
#define BPWM_WGCTL0_PRDPCTL2_Msk        (0x3ul << BPWM_WGCTL0_PRDPCTL2_Pos)              /*!< BPWM_T::WGCTL0: PRDPCTL2 Mask         */

#define BPWM_WGCTL0_PRDPCTL3_Pos        (22)                                              /*!< BPWM_T::WGCTL0: PRDPCTL3 Position     */
#define BPWM_WGCTL0_PRDPCTL3_Msk        (0x3ul << BPWM_WGCTL0_PRDPCTL3_Pos)              /*!< BPWM_T::WGCTL0: PRDPCTL3 Mask         */

#define BPWM_WGCTL0_PRDPCTL4_Pos        (24)                                              /*!< BPWM_T::WGCTL0: PRDPCTL4 Position     */
#define BPWM_WGCTL0_PRDPCTL4_Msk        (0x3ul << BPWM_WGCTL0_PRDPCTL4_Pos)              /*!< BPWM_T::WGCTL0: PRDPCTL4 Mask         */

#define BPWM_WGCTL0_PRDPCTL5_Pos        (26)                                              /*!< BPWM_T::WGCTL0: PRDPCTL5 Position     */
#define BPWM_WGCTL0_PRDPCTL5_Msk        (0x3ul << BPWM_WGCTL0_PRDPCTL5_Pos)              /*!< BPWM_T::WGCTL0: PRDPCTL5 Mask         */

#define BPWM_WGCTL1_CMPUCTL0_Pos        (0)                                               /*!< BPWM_T::WGCTL1: CMPUCTL0 Position     */
#define BPWM_WGCTL1_CMPUCTL0_Msk        (0x3ul << BPWM_WGCTL1_CMPUCTL0_Pos)              /*!< BPWM_T::WGCTL1: CMPUCTL0 Mask         */

#define BPWM_WGCTL1_CMPUCTL1_Pos        (2)                                               /*!< BPWM_T::WGCTL1: CMPUCTL1 Position     */
#define BPWM_WGCTL1_CMPUCTL1_Msk        (0x3ul << BPWM_WGCTL1_CMPUCTL1_Pos)              /*!< BPWM_T::WGCTL1: CMPUCTL1 Mask         */

#define BPWM_WGCTL1_CMPUCTL2_Pos        (4)                                               /*!< BPWM_T::WGCTL1: CMPUCTL2 Position     */
#define BPWM_WGCTL1_CMPUCTL2_Msk        (0x3ul << BPWM_WGCTL1_CMPUCTL2_Pos)              /*!< BPWM_T::WGCTL1: CMPUCTL2 Mask         */

#define BPWM_WGCTL1_CMPUCTL3_Pos        (6)                                               /*!< BPWM_T::WGCTL1: CMPUCTL3 Position     */
#define BPWM_WGCTL1_CMPUCTL3_Msk        (0x3ul << BPWM_WGCTL1_CMPUCTL3_Pos)              /*!< BPWM_T::WGCTL1: CMPUCTL3 Mask         */

#define BPWM_WGCTL1_CMPUCTL4_Pos        (8)                                               /*!< BPWM_T::WGCTL1: CMPUCTL4 Position     */
#define BPWM_WGCTL1_CMPUCTL4_Msk        (0x3ul << BPWM_WGCTL1_CMPUCTL4_Pos)              /*!< BPWM_T::WGCTL1: CMPUCTL4 Mask         */

#define BPWM_WGCTL1_CMPUCTL5_Pos        (10)                                              /*!< BPWM_T::WGCTL1: CMPUCTL5 Position     */
#define BPWM_WGCTL1_CMPUCTL5_Msk        (0x3ul << BPWM_WGCTL1_CMPUCTL5_Pos)              /*!< BPWM_T::WGCTL1: CMPUCTL5 Mask         */

#define BPWM_WGCTL1_CMPDCTL0_Pos        (16)                                              /*!< BPWM_T::WGCTL1: CMPDCTL0 Position     */
#define BPWM_WGCTL1_CMPDCTL0_Msk        (0x3ul << BPWM_WGCTL1_CMPDCTL0_Pos)              /*!< BPWM_T::WGCTL1: CMPDCTL0 Mask         */

#define BPWM_WGCTL1_CMPDCTL1_Pos        (18)                                              /*!< BPWM_T::WGCTL1: CMPDCTL1 Position     */
#define BPWM_WGCTL1_CMPDCTL1_Msk        (0x3ul << BPWM_WGCTL1_CMPDCTL1_Pos)              /*!< BPWM_T::WGCTL1: CMPDCTL1 Mask         */

#define BPWM_WGCTL1_CMPDCTL2_Pos        (20)                                              /*!< BPWM_T::WGCTL1: CMPDCTL2 Position     */
#define BPWM_WGCTL1_CMPDCTL2_Msk        (0x3ul << BPWM_WGCTL1_CMPDCTL2_Pos)              /*!< BPWM_T::WGCTL1: CMPDCTL2 Mask         */

#define BPWM_WGCTL1_CMPDCTL3_Pos        (22)                                              /*!< BPWM_T::WGCTL1: CMPDCTL3 Position     */
#define BPWM_WGCTL1_CMPDCTL3_Msk        (0x3ul << BPWM_WGCTL1_CMPDCTL3_Pos)              /*!< BPWM_T::WGCTL1: CMPDCTL3 Mask         */

#define BPWM_WGCTL1_CMPDCTL4_Pos        (24)                                              /*!< BPWM_T::WGCTL1: CMPDCTL4 Position     */
#define BPWM_WGCTL1_CMPDCTL4_Msk        (0x3ul << BPWM_WGCTL1_CMPDCTL4_Pos)              /*!< BPWM_T::WGCTL1: CMPDCTL4 Mask         */

#define BPWM_WGCTL1_CMPDCTL5_Pos        (26)                                              /*!< BPWM_T::WGCTL1: CMPDCTL5 Position     */
#define BPWM_WGCTL1_CMPDCTL5_Msk        (0x3ul << BPWM_WGCTL1_CMPDCTL5_Pos)              /*!< BPWM_T::WGCTL1: CMPDCTL5 Mask         */

#define BPWM_MSKEN_MSKEN0_Pos           (0)                                               /*!< BPWM_T::MSKEN: MSKEN0 Position        */
#define BPWM_MSKEN_MSKEN0_Msk           (0x1ul << BPWM_MSKEN_MSKEN0_Pos)                 /*!< BPWM_T::MSKEN: MSKEN0 Mask            */

#define BPWM_MSKEN_MSKEN1_Pos           (1)                                               /*!< BPWM_T::MSKEN: MSKEN1 Position        */
#define BPWM_MSKEN_MSKEN1_Msk           (0x1ul << BPWM_MSKEN_MSKEN1_Pos)                 /*!< BPWM_T::MSKEN: MSKEN1 Mask            */

#define BPWM_MSKEN_MSKEN2_Pos           (2)                                               /*!< BPWM_T::MSKEN: MSKEN2 Position        */
#define BPWM_MSKEN_MSKEN2_Msk           (0x1ul << BPWM_MSKEN_MSKEN2_Pos)                 /*!< BPWM_T::MSKEN: MSKEN2 Mask            */

#define BPWM_MSKEN_MSKEN3_Pos           (3)                                               /*!< BPWM_T::MSKEN: MSKEN3 Position        */
#define BPWM_MSKEN_MSKEN3_Msk           (0x1ul << BPWM_MSKEN_MSKEN3_Pos)                 /*!< BPWM_T::MSKEN: MSKEN3 Mask            */

#define BPWM_MSKEN_MSKEN4_Pos           (4)                                               /*!< BPWM_T::MSKEN: MSKEN4 Position        */
#define BPWM_MSKEN_MSKEN4_Msk           (0x1ul << BPWM_MSKEN_MSKEN4_Pos)                 /*!< BPWM_T::MSKEN: MSKEN4 Mask            */

#define BPWM_MSKEN_MSKEN5_Pos           (5)                                               /*!< BPWM_T::MSKEN: MSKEN5 Position        */
#define BPWM_MSKEN_MSKEN5_Msk           (0x1ul << BPWM_MSKEN_MSKEN5_Pos)                 /*!< BPWM_T::MSKEN: MSKEN5 Mask            */

#define BPWM_MSK_MSKDAT0_Pos            (0)                                               /*!< BPWM_T::MSK: MSKDAT0 Position         */
#define BPWM_MSK_MSKDAT0_Msk            (0x1ul << BPWM_MSK_MSKDAT0_Pos)                  /*!< BPWM_T::MSK: MSKDAT0 Mask             */

#define BPWM_MSK_MSKDAT1_Pos            (1)                                               /*!< BPWM_T::MSK: MSKDAT1 Position         */
#define BPWM_MSK_MSKDAT1_Msk            (0x1ul << BPWM_MSK_MSKDAT1_Pos)                  /*!< BPWM_T::MSK: MSKDAT1 Mask             */

#define BPWM_MSK_MSKDAT2_Pos            (2)                                               /*!< BPWM_T::MSK: MSKDAT2 Position         */
#define BPWM_MSK_MSKDAT2_Msk            (0x1ul << BPWM_MSK_MSKDAT2_Pos)                  /*!< BPWM_T::MSK: MSKDAT2 Mask             */

#define BPWM_MSK_MSKDAT3_Pos            (3)                                               /*!< BPWM_T::MSK: MSKDAT3 Position         */
#define BPWM_MSK_MSKDAT3_Msk            (0x1ul << BPWM_MSK_MSKDAT3_Pos)                  /*!< BPWM_T::MSK: MSKDAT3 Mask             */

#define BPWM_MSK_MSKDAT4_Pos            (4)                                               /*!< BPWM_T::MSK: MSKDAT4 Position         */
#define BPWM_MSK_MSKDAT4_Msk            (0x1ul << BPWM_MSK_MSKDAT4_Pos)                  /*!< BPWM_T::MSK: MSKDAT4 Mask             */

#define BPWM_MSK_MSKDAT5_Pos            (5)                                               /*!< BPWM_T::MSK: MSKDAT5 Position         */
#define BPWM_MSK_MSKDAT5_Msk            (0x1ul << BPWM_MSK_MSKDAT5_Pos)                  /*!< BPWM_T::MSK: MSKDAT5 Mask             */

#define BPWM_POLCTL_PINV0_Pos           (0)                                               /*!< BPWM_T::POLCTL: PINV0 Position        */
#define BPWM_POLCTL_PINV0_Msk           (0x1ul << BPWM_POLCTL_PINV0_Pos)                 /*!< BPWM_T::POLCTL: PINV0 Mask            */

#define BPWM_POLCTL_PINV1_Pos           (1)                                               /*!< BPWM_T::POLCTL: PINV1 Position        */
#define BPWM_POLCTL_PINV1_Msk           (0x1ul << BPWM_POLCTL_PINV1_Pos)                 /*!< BPWM_T::POLCTL: PINV1 Mask            */

#define BPWM_POLCTL_PINV2_Pos           (2)                                               /*!< BPWM_T::POLCTL: PINV2 Position        */
#define BPWM_POLCTL_PINV2_Msk           (0x1ul << BPWM_POLCTL_PINV2_Pos)                 /*!< BPWM_T::POLCTL: PINV2 Mask            */

#define BPWM_POLCTL_PINV3_Pos           (3)                                               /*!< BPWM_T::POLCTL: PINV3 Position        */
#define BPWM_POLCTL_PINV3_Msk           (0x1ul << BPWM_POLCTL_PINV3_Pos)                 /*!< BPWM_T::POLCTL: PINV3 Mask            */

#define BPWM_POLCTL_PINV4_Pos           (4)                                               /*!< BPWM_T::POLCTL: PINV4 Position        */
#define BPWM_POLCTL_PINV4_Msk           (0x1ul << BPWM_POLCTL_PINV4_Pos)                 /*!< BPWM_T::POLCTL: PINV4 Mask            */

#define BPWM_POLCTL_PINV5_Pos           (5)                                               /*!< BPWM_T::POLCTL: PINV5 Position        */
#define BPWM_POLCTL_PINV5_Msk           (0x1ul << BPWM_POLCTL_PINV5_Pos)                 /*!< BPWM_T::POLCTL: PINV5 Mask            */

#define BPWM_POEN_POEN0_Pos             (0)                                               /*!< BPWM_T::POEN: POEN0 Position          */
#define BPWM_POEN_POEN0_Msk             (0x1ul << BPWM_POEN_POEN0_Pos)                   /*!< BPWM_T::POEN: POEN0 Mask              */

#define BPWM_POEN_POEN1_Pos             (1)                                               /*!< BPWM_T::POEN: POEN1 Position          */
#define BPWM_POEN_POEN1_Msk             (0x1ul << BPWM_POEN_POEN1_Pos)                   /*!< BPWM_T::POEN: POEN1 Mask              */

#define BPWM_POEN_POEN2_Pos             (2)                                               /*!< BPWM_T::POEN: POEN2 Position          */
#define BPWM_POEN_POEN2_Msk             (0x1ul << BPWM_POEN_POEN2_Pos)                   /*!< BPWM_T::POEN: POEN2 Mask              */

#define BPWM_POEN_POEN3_Pos             (3)                                               /*!< BPWM_T::POEN: POEN3 Position          */
#define BPWM_POEN_POEN3_Msk             (0x1ul << BPWM_POEN_POEN3_Pos)                   /*!< BPWM_T::POEN: POEN3 Mask              */

#define BPWM_POEN_POEN4_Pos             (4)                                               /*!< BPWM_T::POEN: POEN4 Position          */
#define BPWM_POEN_POEN4_Msk             (0x1ul << BPWM_POEN_POEN4_Pos)                   /*!< BPWM_T::POEN: POEN4 Mask              */

#define BPWM_POEN_POEN5_Pos             (5)                                               /*!< BPWM_T::POEN: POEN5 Position          */
#define BPWM_POEN_POEN5_Msk             (0x1ul << BPWM_POEN_POEN5_Pos)                   /*!< BPWM_T::POEN: POEN5 Mask              */

#define BPWM_INTEN_ZIEN0_Pos            (0)                                               /*!< BPWM_T::INTEN: ZIEN0 Position         */
#define BPWM_INTEN_ZIEN0_Msk            (0x1ul << BPWM_INTEN_ZIEN0_Pos)                  /*!< BPWM_T::INTEN: ZIEN0 Mask             */

#define BPWM_INTEN_PIEN0_Pos            (8)                                               /*!< BPWM_T::INTEN: PIEN0 Position         */
#define BPWM_INTEN_PIEN0_Msk            (0x1ul << BPWM_INTEN_PIEN0_Pos)                  /*!< BPWM_T::INTEN: PIEN0 Mask             */

#define BPWM_INTEN_CMPUIEN0_Pos         (16)                                              /*!< BPWM_T::INTEN: CMPUIEN0 Position      */
#define BPWM_INTEN_CMPUIEN0_Msk         (0x1ul << BPWM_INTEN_CMPUIEN0_Pos)               /*!< BPWM_T::INTEN: CMPUIEN0 Mask          */

#define BPWM_INTEN_CMPUIEN1_Pos         (17)                                              /*!< BPWM_T::INTEN: CMPUIEN1 Position      */
#define BPWM_INTEN_CMPUIEN1_Msk         (0x1ul << BPWM_INTEN_CMPUIEN1_Pos)               /*!< BPWM_T::INTEN: CMPUIEN1 Mask          */

#define BPWM_INTEN_CMPUIEN2_Pos         (18)                                              /*!< BPWM_T::INTEN: CMPUIEN2 Position      */
#define BPWM_INTEN_CMPUIEN2_Msk         (0x1ul << BPWM_INTEN_CMPUIEN2_Pos)               /*!< BPWM_T::INTEN: CMPUIEN2 Mask          */

#define BPWM_INTEN_CMPUIEN3_Pos         (19)                                              /*!< BPWM_T::INTEN: CMPUIEN3 Position      */
#define BPWM_INTEN_CMPUIEN3_Msk         (0x1ul << BPWM_INTEN_CMPUIEN3_Pos)               /*!< BPWM_T::INTEN: CMPUIEN3 Mask          */

#define BPWM_INTEN_CMPUIEN4_Pos         (20)                                              /*!< BPWM_T::INTEN: CMPUIEN4 Position      */
#define BPWM_INTEN_CMPUIEN4_Msk         (0x1ul << BPWM_INTEN_CMPUIEN4_Pos)               /*!< BPWM_T::INTEN: CMPUIEN4 Mask          */

#define BPWM_INTEN_CMPUIEN5_Pos         (21)                                              /*!< BPWM_T::INTEN: CMPUIEN5 Position      */
#define BPWM_INTEN_CMPUIEN5_Msk         (0x1ul << BPWM_INTEN_CMPUIEN5_Pos)               /*!< BPWM_T::INTEN: CMPUIEN5 Mask          */

#define BPWM_INTEN_CMPDIEN0_Pos         (24)                                              /*!< BPWM_T::INTEN: CMPDIEN0 Position      */
#define BPWM_INTEN_CMPDIEN0_Msk         (0x1ul << BPWM_INTEN_CMPDIEN0_Pos)               /*!< BPWM_T::INTEN: CMPDIEN0 Mask          */

#define BPWM_INTEN_CMPDIEN1_Pos         (25)                                              /*!< BPWM_T::INTEN: CMPDIEN1 Position      */
#define BPWM_INTEN_CMPDIEN1_Msk         (0x1ul << BPWM_INTEN_CMPDIEN1_Pos)               /*!< BPWM_T::INTEN: CMPDIEN1 Mask          */

#define BPWM_INTEN_CMPDIEN2_Pos         (26)                                              /*!< BPWM_T::INTEN: CMPDIEN2 Position      */
#define BPWM_INTEN_CMPDIEN2_Msk         (0x1ul << BPWM_INTEN_CMPDIEN2_Pos)               /*!< BPWM_T::INTEN: CMPDIEN2 Mask          */

#define BPWM_INTEN_CMPDIEN3_Pos         (27)                                              /*!< BPWM_T::INTEN: CMPDIEN3 Position      */
#define BPWM_INTEN_CMPDIEN3_Msk         (0x1ul << BPWM_INTEN_CMPDIEN3_Pos)               /*!< BPWM_T::INTEN: CMPDIEN3 Mask          */

#define BPWM_INTEN_CMPDIEN4_Pos         (28)                                              /*!< BPWM_T::INTEN: CMPDIEN4 Position      */
#define BPWM_INTEN_CMPDIEN4_Msk         (0x1ul << BPWM_INTEN_CMPDIEN4_Pos)               /*!< BPWM_T::INTEN: CMPDIEN4 Mask          */

#define BPWM_INTEN_CMPDIEN5_Pos         (29)                                              /*!< BPWM_T::INTEN: CMPDIEN5 Position      */
#define BPWM_INTEN_CMPDIEN5_Msk         (0x1ul << BPWM_INTEN_CMPDIEN5_Pos)               /*!< BPWM_T::INTEN: CMPDIEN5 Mask          */

#define BPWM_INTSTS_ZIF0_Pos            (0)                                               /*!< BPWM_T::INTSTS: ZIF0 Position         */
#define BPWM_INTSTS_ZIF0_Msk            (0x1ul << BPWM_INTSTS_ZIF0_Pos)                  /*!< BPWM_T::INTSTS: ZIF0 Mask             */

#define BPWM_INTSTS_PIF0_Pos            (8)                                               /*!< BPWM_T::INTSTS: PIF0 Position         */
#define BPWM_INTSTS_PIF0_Msk            (0x1ul << BPWM_INTSTS_PIF0_Pos)                  /*!< BPWM_T::INTSTS: PIF0 Mask             */

#define BPWM_INTSTS_CMPUIF0_Pos         (16)                                              /*!< BPWM_T::INTSTS: CMPUIF0 Position      */
#define BPWM_INTSTS_CMPUIF0_Msk         (0x1ul << BPWM_INTSTS_CMPUIF0_Pos)               /*!< BPWM_T::INTSTS: CMPUIF0 Mask          */

#define BPWM_INTSTS_CMPUIF1_Pos         (17)                                              /*!< BPWM_T::INTSTS: CMPUIF1 Position      */
#define BPWM_INTSTS_CMPUIF1_Msk         (0x1ul << BPWM_INTSTS_CMPUIF1_Pos)               /*!< BPWM_T::INTSTS: CMPUIF1 Mask          */

#define BPWM_INTSTS_CMPUIF2_Pos         (18)                                              /*!< BPWM_T::INTSTS: CMPUIF2 Position      */
#define BPWM_INTSTS_CMPUIF2_Msk         (0x1ul << BPWM_INTSTS_CMPUIF2_Pos)               /*!< BPWM_T::INTSTS: CMPUIF2 Mask          */

#define BPWM_INTSTS_CMPUIF3_Pos         (19)                                              /*!< BPWM_T::INTSTS: CMPUIF3 Position      */
#define BPWM_INTSTS_CMPUIF3_Msk         (0x1ul << BPWM_INTSTS_CMPUIF3_Pos)               /*!< BPWM_T::INTSTS: CMPUIF3 Mask          */

#define BPWM_INTSTS_CMPUIF4_Pos         (20)                                              /*!< BPWM_T::INTSTS: CMPUIF4 Position      */
#define BPWM_INTSTS_CMPUIF4_Msk         (0x1ul << BPWM_INTSTS_CMPUIF4_Pos)               /*!< BPWM_T::INTSTS: CMPUIF4 Mask          */

#define BPWM_INTSTS_CMPUIF5_Pos         (21)                                              /*!< BPWM_T::INTSTS: CMPUIF5 Position      */
#define BPWM_INTSTS_CMPUIF5_Msk         (0x1ul << BPWM_INTSTS_CMPUIF5_Pos)               /*!< BPWM_T::INTSTS: CMPUIF5 Mask          */

#define BPWM_INTSTS_CMPDIF0_Pos         (24)                                              /*!< BPWM_T::INTSTS: CMPDIF0 Position      */
#define BPWM_INTSTS_CMPDIF0_Msk         (0x1ul << BPWM_INTSTS_CMPDIF0_Pos)               /*!< BPWM_T::INTSTS: CMPDIF0 Mask          */

#define BPWM_INTSTS_CMPDIF1_Pos         (25)                                              /*!< BPWM_T::INTSTS: CMPDIF1 Position      */
#define BPWM_INTSTS_CMPDIF1_Msk         (0x1ul << BPWM_INTSTS_CMPDIF1_Pos)               /*!< BPWM_T::INTSTS: CMPDIF1 Mask          */

#define BPWM_INTSTS_CMPDIF2_Pos         (26)                                              /*!< BPWM_T::INTSTS: CMPDIF2 Position      */
#define BPWM_INTSTS_CMPDIF2_Msk         (0x1ul << BPWM_INTSTS_CMPDIF2_Pos)               /*!< BPWM_T::INTSTS: CMPDIF2 Mask          */

#define BPWM_INTSTS_CMPDIF3_Pos         (27)                                              /*!< BPWM_T::INTSTS: CMPDIF3 Position      */
#define BPWM_INTSTS_CMPDIF3_Msk         (0x1ul << BPWM_INTSTS_CMPDIF3_Pos)               /*!< BPWM_T::INTSTS: CMPDIF3 Mask          */

#define BPWM_INTSTS_CMPDIF4_Pos         (28)                                              /*!< BPWM_T::INTSTS: CMPDIF4 Position      */
#define BPWM_INTSTS_CMPDIF4_Msk         (0x1ul << BPWM_INTSTS_CMPDIF4_Pos)               /*!< BPWM_T::INTSTS: CMPDIF4 Mask          */

#define BPWM_INTSTS_CMPDIF5_Pos         (29)                                              /*!< BPWM_T::INTSTS: CMPDIF5 Position      */
#define BPWM_INTSTS_CMPDIF5_Msk         (0x1ul << BPWM_INTSTS_CMPDIF5_Pos)               /*!< BPWM_T::INTSTS: CMPDIF5 Mask          */

#define BPWM_DACTRGEN_ZTE_Pos            (0)                                               /*!< BPWM_T::DACTRGEN: ZTE Position         */
#define BPWM_DACTRGEN_ZTE_Msk            (0x1ul << BPWM_DACTRGEN_ZTE_Pos)                  /*!< BPWM_T::DACTRGEN: ZTE Mask             */

#define BPWM_DACTRGEN_PTE_Pos            (8)                                               /*!< BPWM_T::DACTRGEN: PTE Position         */
#define BPWM_DACTRGEN_PTE_Msk            (0x1ul << BPWM_DACTRGEN_PTE_Pos)                  /*!< BPWM_T::DACTRGEN: PTE Mask             */

#define BPWM_DACTRGEN_CUTRGE0_Pos        (16)                                              /*!< BPWM_T::DACTRGEN: CUTRGE0 Position     */
#define BPWM_DACTRGEN_CUTRGE0_Msk        (0x1ul << BPWM_DACTRGEN_CUTRGE0_Pos)              /*!< BPWM_T::DACTRGEN: CUTRGE0 Mask         */

#define BPWM_DACTRGEN_CUTRGE1_Pos        (17)                                              /*!< BPWM_T::DACTRGEN: CUTRGE1 Position     */
#define BPWM_DACTRGEN_CUTRGE1_Msk        (0x1ul << BPWM_DACTRGEN_CUTRGE1_Pos)              /*!< BPWM_T::DACTRGEN: CUTRGE1 Mask         */

#define BPWM_DACTRGEN_CUTRGE2_Pos        (18)                                              /*!< BPWM_T::DACTRGEN: CUTRGE2 Position     */
#define BPWM_DACTRGEN_CUTRGE2_Msk        (0x1ul << BPWM_DACTRGEN_CUTRGE2_Pos)              /*!< BPWM_T::DACTRGEN: CUTRGE2 Mask         */

#define BPWM_DACTRGEN_CUTRGE3_Pos        (19)                                              /*!< BPWM_T::DACTRGEN: CUTRGE3 Position     */
#define BPWM_DACTRGEN_CUTRGE3_Msk        (0x1ul << BPWM_DACTRGEN_CUTRGE3_Pos)              /*!< BPWM_T::DACTRGEN: CUTRGE3 Mask         */

#define BPWM_DACTRGEN_CUTRGE4_Pos        (20)                                              /*!< BPWM_T::DACTRGEN: CUTRGE4 Position     */
#define BPWM_DACTRGEN_CUTRGE4_Msk        (0x1ul << BPWM_DACTRGEN_CUTRGE4_Pos)              /*!< BPWM_T::DACTRGEN: CUTRGE4 Mask         */

#define BPWM_DACTRGEN_CUTRGE5_Pos        (21)                                              /*!< BPWM_T::DACTRGEN: CUTRGE5 Position     */
#define BPWM_DACTRGEN_CUTRGE5_Msk        (0x1ul << BPWM_DACTRGEN_CUTRGE5_Pos)              /*!< BPWM_T::DACTRGEN: CUTRGE5 Mask         */

#define BPWM_DACTRGEN_CDTRGE0_Pos        (24)                                              /*!< BPWM_T::DACTRGEN: CDTRGE0 Position     */
#define BPWM_DACTRGEN_CDTRGE0_Msk        (0x1ul << BPWM_DACTRGEN_CDTRGE0_Pos)              /*!< BPWM_T::DACTRGEN: CDTRGE0 Mask         */

#define BPWM_DACTRGEN_CDTRGE1_Pos        (25)                                              /*!< BPWM_T::DACTRGEN: CDTRGE1 Position     */
#define BPWM_DACTRGEN_CDTRGE1_Msk        (0x1ul << BPWM_DACTRGEN_CDTRGE1_Pos)              /*!< BPWM_T::DACTRGEN: CDTRGE1 Mask         */

#define BPWM_DACTRGEN_CDTRGE2_Pos        (26)                                              /*!< BPWM_T::DACTRGEN: CDTRGE2 Position     */
#define BPWM_DACTRGEN_CDTRGE2_Msk        (0x1ul << BPWM_DACTRGEN_CDTRGE2_Pos)              /*!< BPWM_T::DACTRGEN: CDTRGE2 Mask         */

#define BPWM_DACTRGEN_CDTRGE3_Pos        (27)                                              /*!< BPWM_T::DACTRGEN: CDTRGE3 Position     */
#define BPWM_DACTRGEN_CDTRGE3_Msk        (0x1ul << BPWM_DACTRGEN_CDTRGE3_Pos)              /*!< BPWM_T::DACTRGEN: CDTRGE3 Mask         */

#define BPWM_DACTRGEN_CDTRGE4_Pos        (28)                                              /*!< BPWM_T::DACTRGEN: CDTRGE4 Position     */
#define BPWM_DACTRGEN_CDTRGE4_Msk        (0x1ul << BPWM_DACTRGEN_CDTRGE4_Pos)              /*!< BPWM_T::DACTRGEN: CDTRGE4 Mask         */

#define BPWM_DACTRGEN_CDTRGE5_Pos        (29)                                              /*!< BPWM_T::DACTRGEN: CDTRGE5 Position     */
#define BPWM_DACTRGEN_CDTRGE5_Msk        (0x1ul << BPWM_DACTRGEN_CDTRGE5_Pos)              /*!< BPWM_T::DACTRGEN: CDTRGE5 Mask         */

#define BPWM_ADCTS0_TRGSEL0_Pos          (0)                                               /*!< BPWM_T::ADCTS0: TRGSEL0 Position       */
#define BPWM_ADCTS0_TRGSEL0_Msk          (0xful << BPWM_ADCTS0_TRGSEL0_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL0 Mask           */

#define BPWM_ADCTS0_TRGEN0_Pos           (7)                                               /*!< BPWM_T::ADCTS0: TRGEN0 Position        */
#define BPWM_ADCTS0_TRGEN0_Msk           (0x1ul << BPWM_ADCTS0_TRGEN0_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN0 Mask            */

#define BPWM_ADCTS0_TRGSEL1_Pos          (8)                                               /*!< BPWM_T::ADCTS0: TRGSEL1 Position       */
#define BPWM_ADCTS0_TRGSEL1_Msk          (0xful << BPWM_ADCTS0_TRGSEL1_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL1 Mask           */

#define BPWM_ADCTS0_TRGEN1_Pos           (15)                                              /*!< BPWM_T::ADCTS0: TRGEN1 Position        */
#define BPWM_ADCTS0_TRGEN1_Msk           (0x1ul << BPWM_ADCTS0_TRGEN1_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN1 Mask            */

#define BPWM_ADCTS0_TRGSEL2_Pos          (16)                                              /*!< BPWM_T::ADCTS0: TRGSEL2 Position       */
#define BPWM_ADCTS0_TRGSEL2_Msk          (0xful << BPWM_ADCTS0_TRGSEL2_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL2 Mask           */

#define BPWM_ADCTS0_TRGEN2_Pos           (23)                                              /*!< BPWM_T::ADCTS0: TRGEN2 Position        */
#define BPWM_ADCTS0_TRGEN2_Msk           (0x1ul << BPWM_ADCTS0_TRGEN2_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN2 Mask            */

#define BPWM_ADCTS0_TRGSEL3_Pos          (24)                                              /*!< BPWM_T::ADCTS0: TRGSEL3 Position       */
#define BPWM_ADCTS0_TRGSEL3_Msk          (0xful << BPWM_ADCTS0_TRGSEL3_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL3 Mask           */

#define BPWM_ADCTS0_TRGEN3_Pos           (31)                                              /*!< BPWM_T::ADCTS0: TRGEN3 Position        */
#define BPWM_ADCTS0_TRGEN3_Msk           (0x1ul << BPWM_ADCTS0_TRGEN3_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN3 Mask            */

#define BPWM_ADCTS1_TRGSEL4_Pos          (0)                                               /*!< BPWM_T::ADCTS1: TRGSEL4 Position       */
#define BPWM_ADCTS1_TRGSEL4_Msk          (0xful << BPWM_ADCTS1_TRGSEL4_Pos)                /*!< BPWM_T::ADCTS1: TRGSEL4 Mask           */

#define BPWM_ADCTS1_TRGEN4_Pos           (7)                                               /*!< BPWM_T::ADCTS1: TRGEN4 Position        */
#define BPWM_ADCTS1_TRGEN4_Msk           (0x1ul << BPWM_ADCTS1_TRGEN4_Pos)                 /*!< BPWM_T::ADCTS1: TRGEN4 Mask            */

#define BPWM_ADCTS1_TRGSEL5_Pos          (8)                                               /*!< BPWM_T::ADCTS1: TRGSEL5 Position       */
#define BPWM_ADCTS1_TRGSEL5_Msk          (0xful << BPWM_ADCTS1_TRGSEL5_Pos)                /*!< BPWM_T::ADCTS1: TRGSEL5 Mask           */

#define BPWM_ADCTS1_TRGEN5_Pos           (15)                                              /*!< BPWM_T::ADCTS1: TRGEN5 Position        */
#define BPWM_ADCTS1_TRGEN5_Msk           (0x1ul << BPWM_ADCTS1_TRGEN5_Pos)                 /*!< BPWM_T::ADCTS1: TRGEN5 Mask            */

#define BPWM_STATUS_CNTMAX0_Pos         (0)                                               /*!< BPWM_T::STATUS: CNTMAX0 Position      */
#define BPWM_STATUS_CNTMAX0_Msk         (0x1ul << BPWM_STATUS_CNTMAX0_Pos)               /*!< BPWM_T::STATUS: CNTMAX0 Mask          */

#define BPWM_STATUS_ADCTRG0_Pos          (16)                                              /*!< BPWM_T::STATUS: ADCTRG0 Position       */
#define BPWM_STATUS_ADCTRG0_Msk          (0x1ul << BPWM_STATUS_ADCTRG0_Pos)                /*!< BPWM_T::STATUS: ADCTRG0 Mask           */

#define BPWM_STATUS_ADCTRG1_Pos          (17)                                              /*!< BPWM_T::STATUS: ADCTRG1 Position       */
#define BPWM_STATUS_ADCTRG1_Msk          (0x1ul << BPWM_STATUS_ADCTRG1_Pos)                /*!< BPWM_T::STATUS: ADCTRG1 Mask           */

#define BPWM_STATUS_ADCTRG2_Pos          (18)                                              /*!< BPWM_T::STATUS: ADCTRG2 Position       */
#define BPWM_STATUS_ADCTRG2_Msk          (0x1ul << BPWM_STATUS_ADCTRG2_Pos)                /*!< BPWM_T::STATUS: ADCTRG2 Mask           */

#define BPWM_STATUS_ADCTRG3_Pos          (19)                                              /*!< BPWM_T::STATUS: ADCTRG3 Position       */
#define BPWM_STATUS_ADCTRG3_Msk          (0x1ul << BPWM_STATUS_ADCTRG3_Pos)                /*!< BPWM_T::STATUS: ADCTRG3 Mask           */

#define BPWM_STATUS_ADCTRG4_Pos          (20)                                              /*!< BPWM_T::STATUS: ADCTRG4 Position       */
#define BPWM_STATUS_ADCTRG4_Msk          (0x1ul << BPWM_STATUS_ADCTRG4_Pos)                /*!< BPWM_T::STATUS: ADCTRG4 Mask           */

#define BPWM_STATUS_ADCTRG5_Pos          (21)                                              /*!< BPWM_T::STATUS: ADCTRG5 Position       */
#define BPWM_STATUS_ADCTRG5_Msk          (0x1ul << BPWM_STATUS_ADCTRG5_Pos)                /*!< BPWM_T::STATUS: ADCTRG5 Mask           */

#define BPWM_STATUS_DACTRGF_Pos         (24)                                              /*!< BPWM_T::STATUS: DACTRGF Position     */
#define BPWM_STATUS_DACTRGF_Msk         (0x1ul << BPWM_STATUS_DACTRGF_Pos)               /*!< BPWM_T::STATUS: DACTRGF Mask         */

#define BPWM_CAPINEN_CAPINEN0_Pos       (0)                                               /*!< BPWM_T::CAPINEN: CAPINEN0 Position    */
#define BPWM_CAPINEN_CAPINEN0_Msk       (0x1ul << BPWM_CAPINEN_CAPINEN0_Pos)             /*!< BPWM_T::CAPINEN: CAPINEN0 Mask        */

#define BPWM_CAPINEN_CAPINEN1_Pos       (1)                                               /*!< BPWM_T::CAPINEN: CAPINEN1 Position    */
#define BPWM_CAPINEN_CAPINEN1_Msk       (0x1ul << BPWM_CAPINEN_CAPINEN1_Pos)             /*!< BPWM_T::CAPINEN: CAPINEN1 Mask        */

#define BPWM_CAPINEN_CAPINEN2_Pos       (2)                                               /*!< BPWM_T::CAPINEN: CAPINEN2 Position    */
#define BPWM_CAPINEN_CAPINEN2_Msk       (0x1ul << BPWM_CAPINEN_CAPINEN2_Pos)             /*!< BPWM_T::CAPINEN: CAPINEN2 Mask        */

#define BPWM_CAPINEN_CAPINEN3_Pos       (3)                                               /*!< BPWM_T::CAPINEN: CAPINEN3 Position    */
#define BPWM_CAPINEN_CAPINEN3_Msk       (0x1ul << BPWM_CAPINEN_CAPINEN3_Pos)             /*!< BPWM_T::CAPINEN: CAPINEN3 Mask        */

#define BPWM_CAPINEN_CAPINEN4_Pos       (4)                                               /*!< BPWM_T::CAPINEN: CAPINEN4 Position    */
#define BPWM_CAPINEN_CAPINEN4_Msk       (0x1ul << BPWM_CAPINEN_CAPINEN4_Pos)             /*!< BPWM_T::CAPINEN: CAPINEN4 Mask        */

#define BPWM_CAPINEN_CAPINEN5_Pos       (5)                                               /*!< BPWM_T::CAPINEN: CAPINEN5 Position    */
#define BPWM_CAPINEN_CAPINEN5_Msk       (0x1ul << BPWM_CAPINEN_CAPINEN5_Pos)             /*!< BPWM_T::CAPINEN: CAPINEN5 Mask        */

#define BPWM_CAPCTL_CAPEN0_Pos          (0)                                               /*!< BPWM_T::CAPCTL: CAPEN0 Position       */
#define BPWM_CAPCTL_CAPEN0_Msk          (0x1ul << BPWM_CAPCTL_CAPEN0_Pos)                /*!< BPWM_T::CAPCTL: CAPEN0 Mask           */

#define BPWM_CAPCTL_CAPEN1_Pos          (1)                                               /*!< BPWM_T::CAPCTL: CAPEN1 Position       */
#define BPWM_CAPCTL_CAPEN1_Msk          (0x1ul << BPWM_CAPCTL_CAPEN1_Pos)                /*!< BPWM_T::CAPCTL: CAPEN1 Mask           */

#define BPWM_CAPCTL_CAPEN2_Pos          (2)                                               /*!< BPWM_T::CAPCTL: CAPEN2 Position       */
#define BPWM_CAPCTL_CAPEN2_Msk          (0x1ul << BPWM_CAPCTL_CAPEN2_Pos)                /*!< BPWM_T::CAPCTL: CAPEN2 Mask           */

#define BPWM_CAPCTL_CAPEN3_Pos          (3)                                               /*!< BPWM_T::CAPCTL: CAPEN3 Position       */
#define BPWM_CAPCTL_CAPEN3_Msk          (0x1ul << BPWM_CAPCTL_CAPEN3_Pos)                /*!< BPWM_T::CAPCTL: CAPEN3 Mask           */

#define BPWM_CAPCTL_CAPEN4_Pos          (4)                                               /*!< BPWM_T::CAPCTL: CAPEN4 Position       */
#define BPWM_CAPCTL_CAPEN4_Msk          (0x1ul << BPWM_CAPCTL_CAPEN4_Pos)                /*!< BPWM_T::CAPCTL: CAPEN4 Mask           */

#define BPWM_CAPCTL_CAPEN5_Pos          (5)                                               /*!< BPWM_T::CAPCTL: CAPEN5 Position       */
#define BPWM_CAPCTL_CAPEN5_Msk          (0x1ul << BPWM_CAPCTL_CAPEN5_Pos)                /*!< BPWM_T::CAPCTL: CAPEN5 Mask           */

#define BPWM_CAPCTL_CAPINV0_Pos         (8)                                               /*!< BPWM_T::CAPCTL: CAPINV0 Position      */
#define BPWM_CAPCTL_CAPINV0_Msk         (0x1ul << BPWM_CAPCTL_CAPINV0_Pos)               /*!< BPWM_T::CAPCTL: CAPINV0 Mask          */

#define BPWM_CAPCTL_CAPINV1_Pos         (9)                                               /*!< BPWM_T::CAPCTL: CAPINV1 Position      */
#define BPWM_CAPCTL_CAPINV1_Msk         (0x1ul << BPWM_CAPCTL_CAPINV1_Pos)               /*!< BPWM_T::CAPCTL: CAPINV1 Mask          */

#define BPWM_CAPCTL_CAPINV2_Pos         (10)                                              /*!< BPWM_T::CAPCTL: CAPINV2 Position      */
#define BPWM_CAPCTL_CAPINV2_Msk         (0x1ul << BPWM_CAPCTL_CAPINV2_Pos)               /*!< BPWM_T::CAPCTL: CAPINV2 Mask          */

#define BPWM_CAPCTL_CAPINV3_Pos         (11)                                              /*!< BPWM_T::CAPCTL: CAPINV3 Position      */
#define BPWM_CAPCTL_CAPINV3_Msk         (0x1ul << BPWM_CAPCTL_CAPINV3_Pos)               /*!< BPWM_T::CAPCTL: CAPINV3 Mask          */

#define BPWM_CAPCTL_CAPINV4_Pos         (12)                                              /*!< BPWM_T::CAPCTL: CAPINV4 Position      */
#define BPWM_CAPCTL_CAPINV4_Msk         (0x1ul << BPWM_CAPCTL_CAPINV4_Pos)               /*!< BPWM_T::CAPCTL: CAPINV4 Mask          */

#define BPWM_CAPCTL_CAPINV5_Pos         (13)                                              /*!< BPWM_T::CAPCTL: CAPINV5 Position      */
#define BPWM_CAPCTL_CAPINV5_Msk         (0x1ul << BPWM_CAPCTL_CAPINV5_Pos)               /*!< BPWM_T::CAPCTL: CAPINV5 Mask          */

#define BPWM_CAPCTL_RCRLDEN0_Pos        (16)                                              /*!< BPWM_T::CAPCTL: RCRLDEN0 Position     */
#define BPWM_CAPCTL_RCRLDEN0_Msk        (0x1ul << BPWM_CAPCTL_RCRLDEN0_Pos)              /*!< BPWM_T::CAPCTL: RCRLDEN0 Mask         */

#define BPWM_CAPCTL_RCRLDEN1_Pos        (17)                                              /*!< BPWM_T::CAPCTL: RCRLDEN1 Position     */
#define BPWM_CAPCTL_RCRLDEN1_Msk        (0x1ul << BPWM_CAPCTL_RCRLDEN1_Pos)              /*!< BPWM_T::CAPCTL: RCRLDEN1 Mask         */

#define BPWM_CAPCTL_RCRLDEN2_Pos        (18)                                              /*!< BPWM_T::CAPCTL: RCRLDEN2 Position     */
#define BPWM_CAPCTL_RCRLDEN2_Msk        (0x1ul << BPWM_CAPCTL_RCRLDEN2_Pos)              /*!< BPWM_T::CAPCTL: RCRLDEN2 Mask         */

#define BPWM_CAPCTL_RCRLDEN3_Pos        (19)                                              /*!< BPWM_T::CAPCTL: RCRLDEN3 Position     */
#define BPWM_CAPCTL_RCRLDEN3_Msk        (0x1ul << BPWM_CAPCTL_RCRLDEN3_Pos)              /*!< BPWM_T::CAPCTL: RCRLDEN3 Mask         */

#define BPWM_CAPCTL_RCRLDEN4_Pos        (20)                                              /*!< BPWM_T::CAPCTL: RCRLDEN4 Position     */
#define BPWM_CAPCTL_RCRLDEN4_Msk        (0x1ul << BPWM_CAPCTL_RCRLDEN4_Pos)              /*!< BPWM_T::CAPCTL: RCRLDEN4 Mask         */

#define BPWM_CAPCTL_RCRLDEN5_Pos        (21)                                              /*!< BPWM_T::CAPCTL: RCRLDEN5 Position     */
#define BPWM_CAPCTL_RCRLDEN5_Msk        (0x1ul << BPWM_CAPCTL_RCRLDEN5_Pos)              /*!< BPWM_T::CAPCTL: RCRLDEN5 Mask         */

#define BPWM_CAPCTL_FCRLDEN0_Pos        (24)                                              /*!< BPWM_T::CAPCTL: FCRLDEN0 Position     */
#define BPWM_CAPCTL_FCRLDEN0_Msk        (0x1ul << BPWM_CAPCTL_FCRLDEN0_Pos)              /*!< BPWM_T::CAPCTL: FCRLDEN0 Mask         */

#define BPWM_CAPCTL_FCRLDEN1_Pos        (25)                                              /*!< BPWM_T::CAPCTL: FCRLDEN1 Position     */
#define BPWM_CAPCTL_FCRLDEN1_Msk        (0x1ul << BPWM_CAPCTL_FCRLDEN1_Pos)              /*!< BPWM_T::CAPCTL: FCRLDEN1 Mask         */

#define BPWM_CAPCTL_FCRLDEN2_Pos        (26)                                              /*!< BPWM_T::CAPCTL: FCRLDEN2 Position     */
#define BPWM_CAPCTL_FCRLDEN2_Msk        (0x1ul << BPWM_CAPCTL_FCRLDEN2_Pos)              /*!< BPWM_T::CAPCTL: FCRLDEN2 Mask         */

#define BPWM_CAPCTL_FCRLDEN3_Pos        (27)                                              /*!< BPWM_T::CAPCTL: FCRLDEN3 Position     */
#define BPWM_CAPCTL_FCRLDEN3_Msk        (0x1ul << BPWM_CAPCTL_FCRLDEN3_Pos)              /*!< BPWM_T::CAPCTL: FCRLDEN3 Mask         */

#define BPWM_CAPCTL_FCRLDEN4_Pos        (28)                                              /*!< BPWM_T::CAPCTL: FCRLDEN4 Position     */
#define BPWM_CAPCTL_FCRLDEN4_Msk        (0x1ul << BPWM_CAPCTL_FCRLDEN4_Pos)              /*!< BPWM_T::CAPCTL: FCRLDEN4 Mask         */

#define BPWM_CAPCTL_FCRLDEN5_Pos        (29)                                              /*!< BPWM_T::CAPCTL: FCRLDEN5 Position     */
#define BPWM_CAPCTL_FCRLDEN5_Msk        (0x1ul << BPWM_CAPCTL_FCRLDEN5_Pos)              /*!< BPWM_T::CAPCTL: FCRLDEN5 Mask         */

#define BPWM_CAPSTS_CRIFOV0_Pos         (0)                                               /*!< BPWM_T::CAPSTS: CRIFOV0 Position      */
#define BPWM_CAPSTS_CRIFOV0_Msk         (0x1ul << BPWM_CAPSTS_CRIFOV0_Pos)               /*!< BPWM_T::CAPSTS: CRIFOV0 Mask          */

#define BPWM_CAPSTS_CRIFOV1_Pos         (1)                                               /*!< BPWM_T::CAPSTS: CRIFOV1 Position      */
#define BPWM_CAPSTS_CRIFOV1_Msk         (0x1ul << BPWM_CAPSTS_CRIFOV1_Pos)               /*!< BPWM_T::CAPSTS: CRIFOV1 Mask          */

#define BPWM_CAPSTS_CRIFOV2_Pos         (2)                                               /*!< BPWM_T::CAPSTS: CRIFOV2 Position      */
#define BPWM_CAPSTS_CRIFOV2_Msk         (0x1ul << BPWM_CAPSTS_CRIFOV2_Pos)               /*!< BPWM_T::CAPSTS: CRIFOV2 Mask          */

#define BPWM_CAPSTS_CRIFOV3_Pos         (3)                                               /*!< BPWM_T::CAPSTS: CRIFOV3 Position      */
#define BPWM_CAPSTS_CRIFOV3_Msk         (0x1ul << BPWM_CAPSTS_CRIFOV3_Pos)               /*!< BPWM_T::CAPSTS: CRIFOV3 Mask          */

#define BPWM_CAPSTS_CRIFOV4_Pos         (4)                                               /*!< BPWM_T::CAPSTS: CRIFOV4 Position      */
#define BPWM_CAPSTS_CRIFOV4_Msk         (0x1ul << BPWM_CAPSTS_CRIFOV4_Pos)               /*!< BPWM_T::CAPSTS: CRIFOV4 Mask          */

#define BPWM_CAPSTS_CRIFOV5_Pos         (5)                                               /*!< BPWM_T::CAPSTS: CRIFOV5 Position      */
#define BPWM_CAPSTS_CRIFOV5_Msk         (0x1ul << BPWM_CAPSTS_CRIFOV5_Pos)               /*!< BPWM_T::CAPSTS: CRIFOV5 Mask          */

#define BPWM_CAPSTS_CFIFOV0_Pos         (8)                                               /*!< BPWM_T::CAPSTS: CFIFOV0 Position      */
#define BPWM_CAPSTS_CFIFOV0_Msk         (0x1ul << BPWM_CAPSTS_CFIFOV0_Pos)               /*!< BPWM_T::CAPSTS: CFIFOV0 Mask          */

#define BPWM_CAPSTS_CFIFOV1_Pos         (9)                                               /*!< BPWM_T::CAPSTS: CFIFOV1 Position      */
#define BPWM_CAPSTS_CFIFOV1_Msk         (0x1ul << BPWM_CAPSTS_CFIFOV1_Pos)               /*!< BPWM_T::CAPSTS: CFIFOV1 Mask          */

#define BPWM_CAPSTS_CFIFOV2_Pos         (10)                                              /*!< BPWM_T::CAPSTS: CFIFOV2 Position      */
#define BPWM_CAPSTS_CFIFOV2_Msk         (0x1ul << BPWM_CAPSTS_CFIFOV2_Pos)               /*!< BPWM_T::CAPSTS: CFIFOV2 Mask          */

#define BPWM_CAPSTS_CFIFOV3_Pos         (11)                                              /*!< BPWM_T::CAPSTS: CFIFOV3 Position      */
#define BPWM_CAPSTS_CFIFOV3_Msk         (0x1ul << BPWM_CAPSTS_CFIFOV3_Pos)               /*!< BPWM_T::CAPSTS: CFIFOV3 Mask          */

#define BPWM_CAPSTS_CFIFOV4_Pos         (12)                                              /*!< BPWM_T::CAPSTS: CFIFOV4 Position      */
#define BPWM_CAPSTS_CFIFOV4_Msk         (0x1ul << BPWM_CAPSTS_CFIFOV4_Pos)               /*!< BPWM_T::CAPSTS: CFIFOV4 Mask          */

#define BPWM_CAPSTS_CFIFOV5_Pos         (13)                                              /*!< BPWM_T::CAPSTS: CFIFOV5 Position      */
#define BPWM_CAPSTS_CFIFOV5_Msk         (0x1ul << BPWM_CAPSTS_CFIFOV5_Pos)               /*!< BPWM_T::CAPSTS: CFIFOV5 Mask          */

#define BPWM_RCAPDAT0_RCAPDAT_Pos       (0)                                               /*!< BPWM_T::RCAPDAT0: RCAPDAT Position    */
#define BPWM_RCAPDAT0_RCAPDAT_Msk       (0xfffful << BPWM_RCAPDAT0_RCAPDAT_Pos)          /*!< BPWM_T::RCAPDAT0: RCAPDAT Mask        */

#define BPWM_FCAPDAT0_FCAPDAT_Pos       (0)                                               /*!< BPWM_T::FCAPDAT0: FCAPDAT Position    */
#define BPWM_FCAPDAT0_FCAPDAT_Msk       (0xfffful << BPWM_FCAPDAT0_FCAPDAT_Pos)          /*!< BPWM_T::FCAPDAT0: FCAPDAT Mask        */

#define BPWM_RCAPDAT1_RCAPDAT_Pos       (0)                                               /*!< BPWM_T::RCAPDAT1: RCAPDAT Position    */
#define BPWM_RCAPDAT1_RCAPDAT_Msk       (0xfffful << BPWM_RCAPDAT1_RCAPDAT_Pos)          /*!< BPWM_T::RCAPDAT1: RCAPDAT Mask        */

#define BPWM_FCAPDAT1_FCAPDAT_Pos       (0)                                               /*!< BPWM_T::FCAPDAT1: FCAPDAT Position    */
#define BPWM_FCAPDAT1_FCAPDAT_Msk       (0xfffful << BPWM_FCAPDAT1_FCAPDAT_Pos)          /*!< BPWM_T::FCAPDAT1: FCAPDAT Mask        */

#define BPWM_RCAPDAT2_RCAPDAT_Pos       (0)                                               /*!< BPWM_T::RCAPDAT2: RCAPDAT Position    */
#define BPWM_RCAPDAT2_RCAPDAT_Msk       (0xfffful << BPWM_RCAPDAT2_RCAPDAT_Pos)          /*!< BPWM_T::RCAPDAT2: RCAPDAT Mask        */

#define BPWM_FCAPDAT2_FCAPDAT_Pos       (0)                                               /*!< BPWM_T::FCAPDAT2: FCAPDAT Position    */
#define BPWM_FCAPDAT2_FCAPDAT_Msk       (0xfffful << BPWM_FCAPDAT2_FCAPDAT_Pos)          /*!< BPWM_T::FCAPDAT2: FCAPDAT Mask        */

#define BPWM_RCAPDAT3_RCAPDAT_Pos       (0)                                               /*!< BPWM_T::RCAPDAT3: RCAPDAT Position    */
#define BPWM_RCAPDAT3_RCAPDAT_Msk       (0xfffful << BPWM_RCAPDAT3_RCAPDAT_Pos)          /*!< BPWM_T::RCAPDAT3: RCAPDAT Mask        */

#define BPWM_FCAPDAT3_FCAPDAT_Pos       (0)                                               /*!< BPWM_T::FCAPDAT3: FCAPDAT Position    */
#define BPWM_FCAPDAT3_FCAPDAT_Msk       (0xfffful << BPWM_FCAPDAT3_FCAPDAT_Pos)          /*!< BPWM_T::FCAPDAT3: FCAPDAT Mask        */

#define BPWM_RCAPDAT4_RCAPDAT_Pos       (0)                                               /*!< BPWM_T::RCAPDAT4: RCAPDAT Position    */
#define BPWM_RCAPDAT4_RCAPDAT_Msk       (0xfffful << BPWM_RCAPDAT4_RCAPDAT_Pos)          /*!< BPWM_T::RCAPDAT4: RCAPDAT Mask        */

#define BPWM_FCAPDAT4_FCAPDAT_Pos       (0)                                               /*!< BPWM_T::FCAPDAT4: FCAPDAT Position    */
#define BPWM_FCAPDAT4_FCAPDAT_Msk       (0xfffful << BPWM_FCAPDAT4_FCAPDAT_Pos)          /*!< BPWM_T::FCAPDAT4: FCAPDAT Mask        */

#define BPWM_RCAPDAT5_RCAPDAT_Pos       (0)                                               /*!< BPWM_T::RCAPDAT5: RCAPDAT Position    */
#define BPWM_RCAPDAT5_RCAPDAT_Msk       (0xfffful << BPWM_RCAPDAT5_RCAPDAT_Pos)          /*!< BPWM_T::RCAPDAT5: RCAPDAT Mask        */

#define BPWM_FCAPDAT5_FCAPDAT_Pos       (0)                                               /*!< BPWM_T::FCAPDAT5: FCAPDAT Position    */
#define BPWM_FCAPDAT5_FCAPDAT_Msk       (0xfffful << BPWM_FCAPDAT5_FCAPDAT_Pos)          /*!< BPWM_T::FCAPDAT5: FCAPDAT Mask        */

#define BPWM_CAPIEN_CAPRIENn_Pos        (0)                                               /*!< BPWM_T::CAPIEN: CAPRIENn Position     */
#define BPWM_CAPIEN_CAPRIENn_Msk        (0x3ful << BPWM_CAPIEN_CAPRIENn_Pos)             /*!< BPWM_T::CAPIEN: CAPRIENn Mask         */

#define BPWM_CAPIEN_CAPFIENn_Pos        (8)                                               /*!< BPWM_T::CAPIEN: CAPFIENn Position     */
#define BPWM_CAPIEN_CAPFIENn_Msk        (0x3ful << BPWM_CAPIEN_CAPFIENn_Pos)             /*!< BPWM_T::CAPIEN: CAPFIENn Mask         */

#define BPWM_CAPIF_CAPRIF0_Pos          (0)                                               /*!< BPWM_T::CAPIF: CAPRIF0 Position       */
#define BPWM_CAPIF_CAPRIF0_Msk          (0x1ul << BPWM_CAPIF_CAPRIF0_Pos)                /*!< BPWM_T::CAPIF: CAPRIF0 Mask           */

#define BPWM_CAPIF_CAPRIF1_Pos          (1)                                               /*!< BPWM_T::CAPIF: CAPRIF1 Position       */
#define BPWM_CAPIF_CAPRIF1_Msk          (0x1ul << BPWM_CAPIF_CAPRIF1_Pos)                /*!< BPWM_T::CAPIF: CAPRIF1 Mask           */

#define BPWM_CAPIF_CAPRIF2_Pos          (2)                                               /*!< BPWM_T::CAPIF: CAPRIF2 Position       */
#define BPWM_CAPIF_CAPRIF2_Msk          (0x1ul << BPWM_CAPIF_CAPRIF2_Pos)                /*!< BPWM_T::CAPIF: CAPRIF2 Mask           */

#define BPWM_CAPIF_CAPRIF3_Pos          (3)                                               /*!< BPWM_T::CAPIF: CAPRIF3 Position       */
#define BPWM_CAPIF_CAPRIF3_Msk          (0x1ul << BPWM_CAPIF_CAPRIF3_Pos)                /*!< BPWM_T::CAPIF: CAPRIF3 Mask           */

#define BPWM_CAPIF_CAPRIF4_Pos          (4)                                               /*!< BPWM_T::CAPIF: CAPRIF4 Position       */
#define BPWM_CAPIF_CAPRIF4_Msk          (0x1ul << BPWM_CAPIF_CAPRIF4_Pos)                /*!< BPWM_T::CAPIF: CAPRIF4 Mask           */

#define BPWM_CAPIF_CAPRIF5_Pos          (5)                                               /*!< BPWM_T::CAPIF: CAPRIF5 Position       */
#define BPWM_CAPIF_CAPRIF5_Msk          (0x1ul << BPWM_CAPIF_CAPRIF5_Pos)                /*!< BPWM_T::CAPIF: CAPRIF5 Mask           */

#define BPWM_CAPIF_CAPFIF0_Pos          (8)                                               /*!< BPWM_T::CAPIF: CAPFIF0 Position       */
#define BPWM_CAPIF_CAPFIF0_Msk          (0x1ul << BPWM_CAPIF_CAPFIF0_Pos)                /*!< BPWM_T::CAPIF: CAPFIF0 Mask           */

#define BPWM_CAPIF_CAPFIF1_Pos          (9)                                               /*!< BPWM_T::CAPIF: CAPFIF1 Position       */
#define BPWM_CAPIF_CAPFIF1_Msk          (0x1ul << BPWM_CAPIF_CAPFIF1_Pos)                /*!< BPWM_T::CAPIF: CAPFIF1 Mask           */

#define BPWM_CAPIF_CAPFIF2_Pos          (10)                                              /*!< BPWM_T::CAPIF: CAPFIF2 Position       */
#define BPWM_CAPIF_CAPFIF2_Msk          (0x1ul << BPWM_CAPIF_CAPFIF2_Pos)                /*!< BPWM_T::CAPIF: CAPFIF2 Mask           */

#define BPWM_CAPIF_CAPFIF3_Pos          (11)                                              /*!< BPWM_T::CAPIF: CAPFIF3 Position       */
#define BPWM_CAPIF_CAPFIF3_Msk          (0x1ul << BPWM_CAPIF_CAPFIF3_Pos)                /*!< BPWM_T::CAPIF: CAPFIF3 Mask           */

#define BPWM_CAPIF_CAPFIF4_Pos          (12)                                              /*!< BPWM_T::CAPIF: CAPFIF4 Position       */
#define BPWM_CAPIF_CAPFIF4_Msk          (0x1ul << BPWM_CAPIF_CAPFIF4_Pos)                /*!< BPWM_T::CAPIF: CAPFIF4 Mask           */

#define BPWM_CAPIF_CAPFIF5_Pos          (13)                                              /*!< BPWM_T::CAPIF: CAPFIF5 Position       */
#define BPWM_CAPIF_CAPFIF5_Msk          (0x1ul << BPWM_CAPIF_CAPFIF5_Pos)                /*!< BPWM_T::CAPIF: CAPFIF5 Mask           */

#define BPWM_PBUF_PBUF_Pos              (0)                                               /*!< BPWM_T::PBUF: PBUF Position           */
#define BPWM_PBUF_PBUF_Msk              (0xfffful << BPWM_PBUF_PBUF_Pos)                 /*!< BPWM_T::PBUF: PBUF Mask               */

#define BPWM_CMPBUF_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF0: CMPBUF Position      */
#define BPWM_CMPBUF_CMPBUF_Msk         (0xfffful << BPWM_CMPBUFn_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF0: CMPBUF Mask          */

#define BPWM_CMPBUF0_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF0: CMPBUF Position      */
#define BPWM_CMPBUF0_CMPBUF_Msk         (0xfffful << BPWM_CMPBUF0_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF0: CMPBUF Mask          */

#define BPWM_CMPBUF1_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF1: CMPBUF Position      */
#define BPWM_CMPBUF1_CMPBUF_Msk         (0xfffful << BPWM_CMPBUF1_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF1: CMPBUF Mask          */

#define BPWM_CMPBUF2_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF2: CMPBUF Position      */
#define BPWM_CMPBUF2_CMPBUF_Msk         (0xfffful << BPWM_CMPBUF2_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF2: CMPBUF Mask          */

#define BPWM_CMPBUF3_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF3: CMPBUF Position      */
#define BPWM_CMPBUF3_CMPBUF_Msk         (0xfffful << BPWM_CMPBUF3_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF3: CMPBUF Mask          */

#define BPWM_CMPBUF4_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF4: CMPBUF Position      */
#define BPWM_CMPBUF4_CMPBUF_Msk         (0xfffful << BPWM_CMPBUF4_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF4: CMPBUF Mask          */

#define BPWM_CMPBUF5_CMPBUF_Pos         (0)                                               /*!< BPWM_T::CMPBUF5: CMPBUF Position      */
#define BPWM_CMPBUF5_CMPBUF_Msk         (0xfffful << BPWM_CMPBUF5_CMPBUF_Pos)            /*!< BPWM_T::CMPBUF5: CMPBUF Mask          */

/**@}*/ /* BPWM_CONST */
/**@}*/ /* end of BPWM register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __BPWM_REG_H__ */


