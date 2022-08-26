/*
 * File:   UART1.c
 * Author: user
 *
 * Created on April 30, 2020, 9:02 AM
 */

#include "UART1.h"

#include <xc.h>

#include "Common.h"

uint16_t U1Tx_PotVolt_mV[U1TX_MSG_BUF_LENGTH];
uint16_t U1Tx_Protocol[16];
uint16_t U1Rx_BufferA[8];
uint16_t U1Rx_BufferB[8];

uint8_t  U1TxSimMsg[U1TX_MSG_BUF_LENGTH] = "Now is Simulator\r\n";
uint8_t *pU1String;

extern xSwitchItem_t SwitchStatus;

extern uint16_t ADC1BUF[8], ADC1CH0123_mV[4];

void (*UART1_U1InterruptHandler)(void);
void (*UART1_DMA0InterruptHandler)(void);
static void U1CallBack(void);
static void DMA0CallBack(void);

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
    U1CallBack();
    IFS0bits.U1TXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void) {
    DMA0CallBack();
    IFS0bits.DMA0IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void) {
    static uint16_t BufferCount = 0;
    if (BufferCount == 0) {
        DMA0STAL = __builtin_dmaoffset(U1Rx_BufferA);
        DMA0STAH = 0x0000;
    } else {
        DMA0STBL = __builtin_dmaoffset(U1Rx_BufferB);
        DMA0STBH = 0x0000;
    }
    DMA0CONbits.CHEN  = 1;
    DMA0REQbits.FORCE = 1;
    BufferCount ^= 1;
    IFS0bits.DMA1IF = 0;
}

void UART1Initialize(void) {
    uint16_t i, *pS = U1Tx_PotVolt_mV;
    uint8_t  defaultstring[8] = "0000 mV.";
    for (i = 0; i < sizeof(defaultstring); i++) {
        *pS++ = defaultstring[i];
        Nop();
    }
    pU1String         = U1TxSimMsg;
    U1MODE            = 0x8008 & ~(1 << 15);
    U1STA             = 0x0000;
    U1BRG             = U1_BRP_VAL;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN   = 1;
    /* Wait at least X microseconds (1/uart baudrate) before sending first char */
    for (i = 0; i < (FCY / U1_BAUDRATE + 20); i++) {
        Nop();
    }
#ifdef USING_SIMULATOR
    IEC0bits.U1TXIE = 1;  // Enable UART TX interrupt
    UART1_U1SetIntHandler(UART1_U1DefInterruptHandler);
#else
    DMA0_Initialize();
#endif
}

void DMA0_Initialize(void) {
    DMA0CON            = 0x2001;  // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA0CNT            = 7;       // Eight DMA requests
    DMA0REQbits.IRQSEL = 0x000C;  // Select UART1 transmitter
    DMA0PAD            = (volatile uint16_t) & U1TXREG;
    DMA0STAL           = __builtin_dmaoffset(U1Tx_PotVolt_mV);
    DMA0STAH           = 0x0000;

    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    UART1_DMA0SetIntHandler(UART1_DMA0DefInterruptHandler);
}

void DMA1_Initialize(void) {
    DMA1CON          = 0x0002;  // Continuous, Ping-Pong, Post-Inc., Periph-RAM
    DMA1CNT          = 7;       // Eight DMA requests
    DMA1REQ          = 0x000B;  // Select UART1 receiver
    DMA1PAD          = (volatile uint16_t) & U1RXREG;
    DMA1STAL         = __builtin_dmaoffset(U1Rx_BufferA);
    DMA1STAH         = 0x0000;
    DMA1STBL         = __builtin_dmaoffset(U1Rx_BufferB);
    DMA1STBH         = 0x0000;
    IFS0bits.DMA1IF  = 0;
    IEC0bits.DMA1IE  = 1;
    DMA1CONbits.CHEN = 1;
}

void UART1_U1SetIntHandler(void *handler) {
    UART1_U1InterruptHandler = handler;
}

void UART1_DMA0SetIntHandler(void *handler) {
    UART1_DMA0InterruptHandler = handler;
}

static void U1CallBack(void) {
    if (UART1_U1InterruptHandler) {
        UART1_U1InterruptHandler();
    }
}

static void DMA0CallBack() {
    if (UART1_DMA0InterruptHandler) {
        UART1_DMA0InterruptHandler();
    }
}
