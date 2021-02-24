/*
 * File:   UART1.c
 * Author: user
 *
 * Created on 2020年4月30日, 上午 9:02
 */


#include <xc.h>
#include "Common.h"


unsigned int UART1_TX_BUFFER[UART1_BUFFER_SIZE];
unsigned int UART1_TX_Protocol[16];
unsigned int UART1_RX_BUFFER_A[8];
unsigned int UART1_RX_BUFFER_B[8];

unsigned char UART1_String[16] = "Lioho San DICK!";
unsigned char *UART1_Tx;

extern unsigned int ADCValues_BUFFER[8], ADCValues[8];

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    if (!*UART1_Tx) {
        UART1_Tx = UART1_String;
    }
    U1TXREG = *UART1_Tx++; // Transmit one character
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void) {
    static unsigned int BufferCount = 0; // Keep record of which buffer// contains TX Data
    if (BufferCount == 0) {
        UART1_TX_Protocol[0] = 0X0D;
        UART1_TX_Protocol[1] = ADCValues_BUFFER[1] >> 8;
        UART1_TX_Protocol[2] = ADCValues_BUFFER[1]&0xFF;
        UART1_TX_Protocol[3] = UART1_TX_Protocol[0]^UART1_TX_Protocol[1]^UART1_TX_Protocol[2];
        UART1_TX_Protocol[4] = 0x0C;
        DMA0STAL = __builtin_dmaoffset(UART1_TX_Protocol);
        DMA0STAH = 0x0000;
        DMA0CNT = 4;
    } else {
        UART1_TX_BUFFER[0] = (ADCValues[0] / 1000) + '0';
        UART1_TX_BUFFER[1] = (ADCValues[0] / 100 % 10) + '0';
        UART1_TX_BUFFER[2] = (ADCValues[0] / 10 % 10) + '0';
        UART1_TX_BUFFER[3] = (ADCValues[0] % 10) + '0';
        DMA0STAL = __builtin_dmaoffset(UART1_TX_BUFFER);
        DMA0STAH = 0x0000;
        DMA0CNT = 7;
    }
    BufferCount ^= 1;
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag   
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void) {
    static unsigned int BufferCount = 0; // Keep record of which buffer// contains RX Data
    if (BufferCount == 0) {
        DMA0STAL = __builtin_dmaoffset(UART1_RX_BUFFER_A);
        DMA0STAH = 0x0000;
    } else {
        DMA0STBL = __builtin_dmaoffset(UART1_RX_BUFFER_B);
        DMA0STBH = 0x0000;
    }
    DMA0CONbits.CHEN = 1; // Enable DMA0 channel
    DMA0REQbits.FORCE = 1; // Manual mode:Kick-start the 1st transfer
    BufferCount ^= 1;
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag} 
}

void UART1_Initial(void) { // 115200 bps

    unsigned int i = 0, initValue = 9487;
    UART1_Tx = UART1_String;
    U1MODE = 0x8008 & ~(1 << 15);
    U1STA = 0x0000;

    U1BRG = UART1_BRP_VAL;

    //IEC0bits.U1TXIE = 1; // Enable UART TX interrupt
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;

    /* Wait at least 9 microseconds (1/115200) before sending first char */
    for (i = 0; i < (FCY / UART1_BAUDRATE + 20); i++) {
        Nop();
    }
    UART1_TX_BUFFER[0] = (initValue / 1000) + '0';
    UART1_TX_BUFFER[1] = (initValue / 100 % 10) + '0';
    UART1_TX_BUFFER[2] = (initValue / 10 % 10) + '0';
    UART1_TX_BUFFER[3] = (initValue % 10) + '0';
    UART1_TX_BUFFER[4] = ' ';
    UART1_TX_BUFFER[5] = 'm';
    UART1_TX_BUFFER[6] = 'V';
    UART1_TX_BUFFER[7] = '.';
    DMA0Init();
}

void DMA0Init(void) {
    DMA0CON = 0x2001; // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA0CNT = 7; // Eight DMA requests
    DMA0REQbits.IRQSEL = 0x000C; // Select UART1 transmitter
    DMA0PAD = (volatile unsigned int) &U1TXREG;
    DMA0STAL = __builtin_dmaoffset(UART1_TX_BUFFER);
    DMA0STAH = 0x0000;

    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
}

void DMA1Init(void) {
    DMA1CON = 0x0002; // Continuous, Ping-Pong, Post-Inc., Periph-RAM
    DMA1CNT = 7; // Eight DMA requests
    DMA1REQ = 0x000B; // Select UART1 receiver
    DMA1PAD = (volatile unsigned int) &U1RXREG;
    DMA1STAL = __builtin_dmaoffset(UART1_RX_BUFFER_A);
    DMA1STAH = 0x0000;
    DMA1STBL = __builtin_dmaoffset(UART1_RX_BUFFER_B);
    DMA1STBH = 0x0000;
    IFS0bits.DMA1IF = 0; // Clear DMA interrupt
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt
    DMA1CONbits.CHEN = 1; // Enable DMA channel
}
