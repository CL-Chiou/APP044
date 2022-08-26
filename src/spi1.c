
/**
  SPI1 Generated Driver API Source File

  Company:
    Microchip Technology Inc.

  File Name:
    spi1.c

  @Summary
    This is the generated source file for the SPI1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for SPI1.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.167.0
        Device            :  dsPIC33EP512MU810
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.50
        MPLAB             :  MPLAB X v5.35
 */

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

/**
  Section: Included Files
 */

#include "spi1.h"

#include "Common.h"

/**
 Section: File specific functions
 */

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void);
void     SPI1_Exchange(uint8_t *pTransmitData, uint8_t *pReceiveData);
uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData);

/*DMA Buffer*/
uint16_t SPI1_TXBUFFER[16];

volatile uint8_t SPITransmitDone;

void (*SPI1_InterruptHandler)(void);
static void SPI1CallBack(void);

/**
 Section: Driver Interface Function Definitions
 */

void SPI1Initialize(void) {
    IEC0bits.SPI1EIE = 0;
    IEC0bits.SPI1IE  = 0;

    SPI1CON1bits.SPRE = 8 - 4;
    SPI1CON1bits.PPRE = 3;

    SPI1CON1bits.MODE16 = 1;
    SPI1CON1bits.MSTEN  = 1;
    SPI1CON1bits.CKP    = 0;
    SPI1CON1bits.CKE    = 1;

    SPI1CON2bits.FRMEN  = 0;
    SPI1CON2bits.SPIFSD = 0;
    SPI1CON2bits.FRMPOL = 1;
    SPI1CON2bits.FRMDLY = 0;
    SPI1CON2bits.SPIBEN = 0;

    SPI1STATbits.SPISIDL = 0;
    SPI1STATbits.SPIBEC  = 0;
    SPI1STATbits.SPIROV  = 0;
    SPI1STATbits.SPIEN   = 1;

    // Force First Word After Enabling SPI
    /*DMA3REQbits.FORCE = 1;
    while (DMA3REQbits.FORCE);*/

    DMA3_Initialize();

    IFS0bits.SPI1IF  = 0;
    IFS0bits.SPI1EIF = 0;
    IEC0bits.SPI1EIE = 1;
    IEC0bits.SPI1IE  = 1;

    SPI1_SetIntHandler(SPI1_DefInterruptHandler);
}

void SPI1_Exchange(uint8_t *pTransmitData, uint8_t *pReceiveData) {
    while (SPI1STATbits.SPITBF == true) {
    }

    SPI1BUF = *((uint16_t *)pTransmitData);

    while (SPI1STATbits.SRXMPT == true)
        ;

    *((uint16_t *)pReceiveData) = SPI1BUF;
}

uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData) {
    uint16_t dataSentCount     = 0;
    uint16_t dataReceivedCount = 0;
    uint16_t dummyDataReceived = 0;
    uint16_t dummyDataTransmit = SPI1_DUMMY_DATA;

    uint8_t *pSend, *pReceived;
    uint16_t addressIncrement;
    uint16_t receiveAddressIncrement, sendAddressIncrement;

    addressIncrement = 2;
    byteCount >>= 1;

    // set the pointers and increment delta
    // for transmit and receive operations
    if (pTransmitData == NULL) {
        sendAddressIncrement = 0;
        pSend                = (uint8_t *)&dummyDataTransmit;
    } else {
        sendAddressIncrement = addressIncrement;
        pSend                = (uint8_t *)pTransmitData;
    }

    if (pReceiveData == NULL) {
        receiveAddressIncrement = 0;
        pReceived               = (uint8_t *)&dummyDataReceived;
    } else {
        receiveAddressIncrement = addressIncrement;
        pReceived               = (uint8_t *)pReceiveData;
    }

    while (SPI1STATbits.SPITBF == true) {
    }

    while (dataSentCount < byteCount) {
        if (SPI1STATbits.SPITBF != true) {
            SPI1BUF = *((uint16_t *)pSend);

            pSend += sendAddressIncrement;
            dataSentCount++;
        }

        if (SPI1STATbits.SRXMPT == false) {
            *((uint16_t *)pReceived) = SPI1BUF;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }
    }
    while (dataReceivedCount < byteCount) {
        if (SPI1STATbits.SRXMPT == false) {
            *((uint16_t *)pReceived) = SPI1BUF;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }
    }

    return dataSentCount;
}

uint16_t SPI1_Exchange16bit(uint16_t data) {
    uint16_t receiveData;

    SPI1_Exchange((uint8_t *)&data, (uint8_t *)&receiveData);

    return (receiveData);
}

uint16_t SPI1_Exchange16bitBuffer(uint16_t *dataTransmitted, uint16_t byteCount, uint16_t *dataReceived) {
    return (SPI1_ExchangeBuffer((uint8_t *)dataTransmitted, byteCount, (uint8_t *)dataReceived));
}

/**

    The module's transfer mode affects the operation
    of the exchange functions. The table below shows
    the effect on data sent or received:
    |=======================================================================|
    | Transfer Mode  |     Exchange Function      |        Comments         |
    |=======================================================================|
    |                | SPIx_Exchange8bitBuffer()  |                         |
    |                |----------------------------|  OK                     |
    |                | SPIx_Exchange8bit()        |                         |
    |     8 bits     |----------------------------|-------------------------|
    |                | SPIx_Exchange16bitBuffer() | Do not use. Only the    |
    |                |----------------------------| lower byte of the 16-bit|
    |                | SPIx_Exchange16bit()       | data will be sent or    |
    |                |                            | received.               |
    |----------------|----------------------------|-------------------------|
    |                | SPIx_Exchange8bitBuffer()  | Do not use. Additional  |
    |                |----------------------------| data byte will be       |
    |                | SPIx_Exchange8bit()        | inserted for each       |
    |                |                            | 8-bit data.             |
    |     16 bits    |----------------------------|-------------------------|
    |                | SPIx_Exchange16bitBuffer() |                         |
    |                |----------------------------|  OK                     |
    |                | SPIx_Exchange16bit()       |                         |
    |----------------|----------------------------|-------------------------|
 */
inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void) {
    if (SPI1CON1bits.MODE16 == 0)
        return SPI1_DRIVER_TRANSFER_MODE_8BIT;
    else
        return SPI1_DRIVER_TRANSFER_MODE_16BIT;
}

SPI1_STATUS SPI1_StatusGet() {
    return (SPI1STAT);
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void) {
    SPI1CallBack();
    IFS0bits.SPI1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1ErrInterrupt(void) {
    if (SPI1STATbits.SPIROV == 1) {
        SPI1STATbits.SPIROV = 0;
    }
    IFS0bits.SPI1EIF = 0;
}

void DMA3_Initialize(void) {
    IFS2bits.DMA3IF = 0;  // Clear DMA Interrupt Flag
    IEC2bits.DMA3IE = 1;  // Enable DMA interrupt

    DMA3CON            = 0x2001;      // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA3CNT            = 0;           // Fifteen DMA requests
    DMA3REQbits.IRQSEL = 0b00001010;  // Select SPI1 Transfer Done
    DMA3PAD            = (volatile uint16_t) & SPI1BUF;

    DMA3STAL = __builtin_dmaoffset(SPI1_TXBUFFER);
    DMA3STAH = 0x0000;
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA3Interrupt(void) {
    static uint16_t BufferCount = 0;
    SPITransmitDone             = 1;
    BufferCount ^= 1;
    IFS2bits.DMA3IF = 0;  // Clear the DMA3 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA4Interrupt(void) {
    IFS2bits.DMA4IF = 0;  // Clear the DMA4 Interrupt Flag
}

void SPI1_SetIntHandler(void *handler) {
    SPI1_InterruptHandler = handler;
}

static void SPI1CallBack(void) {
    if (SPI1_InterruptHandler) {
        SPI1_InterruptHandler();
    }
}
/**
 End of File
 */