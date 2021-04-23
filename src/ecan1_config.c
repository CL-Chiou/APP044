/*******************************************************************************
  ECAN1 Configuration source file

  Company:
    Microchip Technology Inc.

  File Name:
    ecan1_config.c

  Summary:
    Initializes and configures the ECAN1 and DMA modules.
    
  Description:
    This source file initializes the DMA and configures two DMA channels one
    each for transmission and reception. The ECAN is also initialized, its clock
    configured to be the system clock itself and the filters are also configured
    to accept a particular message.
 *******************************************************************************/
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "ecan1_config.h"
#include "common.h"

/******************************************************************************
 * Function:      void DMA5_Initialize(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      DMA5 initialization/configuration function.
 *                Direction: Read from RAM and write to the C1TXD register
 *                AMODE: Peripheral Indirect Addressing mode
 *                MODE: Continuous, Ping-Pong modes disabled
 *                IRQ: ECAN1 Transmit Interrupt
 *****************************************************************************/
void DMA5_Initialize(void) {
    DMA5CON = 0x2020;
    DMA5PAD = (volatile uint16_t) &C1TXD; /* ECAN 1 (C1TXD) */
    DMA5CNT = 0x0007;
    DMA5REQ = 0x0046; /* ECAN 1 Transmit */

    DMA5STAL = (uint16_t) (int_least24_t) (&ecan1msgBuf);
    DMA5STAH = 0;

    DMA5CONbits.CHEN = 1;
}

/******************************************************************************
 * Function:      void DMA6_Initialize(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      DMA6 initialization/configuration function.
 *                Direction: Read from RAM and write to the C1RXD register
 *                AMODE: Peripheral Indirect Addressing mode
 *                MODE: Continuous, Ping-Pong modes disabled
 *                IRQ: ECAN1 Transmit Interrupt
 *****************************************************************************/
void DMA6_Initialize(void) {
    DMA6CON = 0x0020;
    DMA6PAD = (volatile uint16_t) &C1RXD; /* ECAN 1 (C1RXD) */
    DMA6CNT = 0x0007;
    DMA6REQ = 0x0022; /* ECAN 1 Receive */

    DMA6STAL = (uint16_t) (int_least24_t) (&ecan1msgBuf);
    DMA6STAH = 0;
    DMA6CONbits.CHEN = 1;
}

/******************************************************************************
 * Function:      void Ecan1ClkInit(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      ECAN1 clock initialization function
 *                This function is used to configure the clock used for the
 *                ECAN1 module during transmission/reception.
 *****************************************************************************/
void Ecan1ClkInit(void) {
    /*  FCAN is selected to be FCY
            FCAN = FCY = 60MHz 

    Bit Time = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)=20*TQ = N
    Phase Segment 1 = 8TQ
    Phase Segment 2 = 6Tq
    Propagation Delay = 5Tq
    Sync Segment = 1TQ
    CiCFG1<BRP> =(FCAN /(2 * N/FBAUD))-1

    Definition in ecan1_config.h as below 
    #define CAN1_BRP_VAL ( (FCAN / (2 * NTQ * CAN1_BITRATE)) - 1 )
     */

    /* Synchronization Jump Width set to 4 TQ */

    C1CFG1bits.SJW = 0x3;

    /* Baud Rate Prescaler */
    C1CFG1bits.BRP = CAN1_BRP_VAL;

    /* Phase Segment 1 time is 8 TQ */
    C1CFG2bits.SEG1PH = 0x7;

    /* Phase Segment 2 time is set to be programmable */
    C1CFG2bits.SEG2PHTS = 0x1;

    /* Phase Segment 2 time is 6 TQ */
    C1CFG2bits.SEG2PH = 0x5;

    /* Propagation Segment time is 5 TQ */
    C1CFG2bits.PRSEG = 0x4;

    /* Bus line is sampled three times at the sample point */
    C1CFG2bits.SAM = 0x1;
}

/******************************************************************************
 * Function:     void ECAN1_Initialize(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      ECAN1 initialization function.This function is used to
 *                initialize the ECAN1 module by configuring the message
 *                buffers, and the acceptance filters and
 *                setting appropriate masks for the same.
 *****************************************************************************/
void ECAN1_Initialize(void) {
    /* Request Configuration Mode */
    C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);

    C1CTRL1bits.CANCKS = 1;

    Ecan1ClkInit();

    C1FCTRLbits.FSA = 0b01000; /* FIFO Starts at Message Buffer 8 */
    C1FCTRLbits.DMABS = 0b100; /* 16 CAN Message Buffers in DMA RAM */

    /*	Filter Configuration

    Ecan1WriteRxAcptFilter(int n, long identifier, uint16_t exide,uint16_t bufPnt,uint16_t maskSel)

    n = 0 to 15 -> Filter number

    identifier -> SID <10:0> : EID <17:0> 

    exide = 0 -> Match messages with standard identifier addresses 
    exide = 1 -> Match messages with extended identifier addresses 

    bufPnt = 0 to 14  -> RX Buffer 0 to 14
    bufPnt = 15 -> RX FIFO Buffer

    maskSel = 0    ->    Acceptance Mask 0 register contains mask
    maskSel = 1    ->    Acceptance Mask 1 register contains mask
    maskSel = 2    ->    Acceptance Mask 2 register contains mask
    maskSel = 3    ->    No Mask Selection
    
     */

    Ecan1WriteRxAcptFilter(5, CANRX_ID_1, 1, 04, 0); // Filter 5
    Ecan1WriteRxAcptFilter(6, CANTX_ID_TRIGGER, 1, 06, 1); // Filter 6
    Ecan1WriteRxAcptFilter(9, CANRX_ID_2, 1, 07, 2); // Filter 9
    /*    Mask Configuration
    Ecan1WriteRxAcptMask(int m, long identifierMask, uint16_t mide, uint16_t exide)
    m = 0 to 2 -> Mask Number
    identifier -> SID <10:0> : EID <17:0> 
    mide = 0 -> Match either standard or extended address message if filters match 
    mide = 1 -> Match only message types that correpond to 'exide' bit in filter

    exide = 0 -> Match messages with standard identifier addresses 
    exide = 1 -> Match messages with extended identifier addresses
     */
    Ecan1WriteRxAcptMask(0, 0x01FFFFFFE, 1, 1);
    Ecan1WriteRxAcptMask(1, 0x01FFFFFFF, 1, 1);
    Ecan1WriteRxAcptMask(2, 0x01FFFFFF0, 0, 1);

    /* Enter Normal Mode */
    C1CTRL1bits.REQOP = 0;
#ifndef USING_SIMULATOR
    while (C1CTRL1bits.OPMODE != 0);
#endif

    /* ECAN transmit/receive message control */
    C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x0000;
    C1TR01CONbits.TXEN0 = 1; /* ECAN1, Buffer 0 is a Transmit Buffer */
    C1TR01CONbits.TXEN1 = 1; /* ECAN1, Buffer 1 is a Transmit Buffer */
    C1TR23CONbits.TXEN2 = 1; /* ECAN1, Buffer 2 is a Transmit Buffer */
    C1TR23CONbits.TXEN3 = 1; /* ECAN1, Buffer 3 is a Transmit Buffer */

    C1TR01CONbits.TX0PRI = 0b00; /* Message Buffer 0 Priority Level */
    C1TR01CONbits.TX1PRI = 0b01; /* Message Buffer 1 Priority Level */
    C1TR23CONbits.TX2PRI = 0b10; /* Message Buffer 2 Priority Level */
    C1TR23CONbits.TX3PRI = 0b11; /* Message Buffer 3 Priority Level */

    IEC2bits.C1IE = 1;
    C1INTEbits.TBIE = 1;
    C1INTEbits.RBIE = 1;

    DMA5_Initialize();
    DMA6_Initialize();
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void) {
    IFS3bits.DMA5IF = 0; // Clear the DMA0 Interrupt Flag;
}

/*******************************************************************************
 End of File
 */
