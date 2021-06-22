/*
 * File:   UART1.c
 * Author: user
 *
 * Created on 2020年4月30日, 上午 9:02
 */


#include <xc.h>
#include "Common.h"


uint16_t U1Tx_PotVolt_mV[U1TX_MSG_BUF_LENGTH];
uint16_t U1Tx_Protocol[16];
uint16_t U1Rx_BufferA[8];
uint16_t U1Rx_BufferB[8];

uint8_t U1Tx_SimMsg[U1TX_MSG_BUF_LENGTH] = "Now is Simulator\r\n";
uint8_t *pU1TxAddress;

extern SwitchItem_t SwitchStatus;

extern uint16_t ADC1BUF[8], ChannelVolt_mV[8];

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    if (*pU1TxAddress != 0) {
        U1TXREG = *pU1TxAddress++; // Transmit one character  
    } else {
        Nop();
    }

}

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void) {
    static uint16_t BufferCount = 0; // Keep record of which buffer// contains TX Data
    if (BufferCount == 0) {
        U1Tx_Protocol[0] = 0X0D;
        U1Tx_Protocol[1] = ChannelVolt_mV[0] >> 8;
        U1Tx_Protocol[2] = ChannelVolt_mV[0]&0xFF;
        //U1Tx_SanProtocol[3] = U1Tx_SanProtocol[0]^U1Tx_SanProtocol[1]^U1Tx_SanProtocol[2];
        U1Tx_Protocol[3] = ((U1Tx_Protocol[0]^U1Tx_Protocol[1]^U1Tx_Protocol[2]) + (SwitchStatus.bDIP != 0 ? 1 : 0)) & 0xFF;
        U1Tx_Protocol[4] = 0x0C;
        DMA0STAL = __builtin_dmaoffset(U1Tx_Protocol);
        DMA0STAH = 0x0000;
        DMA0CNT = 4;
    } else {
        U1Tx_PotVolt_mV[0] = (ChannelVolt_mV[0] / 1000) + '0';
        U1Tx_PotVolt_mV[1] = (ChannelVolt_mV[0] / 100 % 10) + '0';
        U1Tx_PotVolt_mV[2] = (ChannelVolt_mV[0] / 10 % 10) + '0';
        U1Tx_PotVolt_mV[3] = (ChannelVolt_mV[0] % 10) + '0';
        DMA0STAL = __builtin_dmaoffset(U1Tx_PotVolt_mV);
        DMA0STAH = 0x0000;
        DMA0CNT = 7;
    }
    BufferCount ^= 1;
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag   
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void) {
    static uint16_t BufferCount = 0; // Keep record of which buffer// contains RX Data
    if (BufferCount == 0) {
        DMA0STAL = __builtin_dmaoffset(U1Rx_BufferA);
        DMA0STAH = 0x0000;
    } else {
        DMA0STBL = __builtin_dmaoffset(U1Rx_BufferB);
        DMA0STBH = 0x0000;
    }
    DMA0CONbits.CHEN = 1; // Enable DMA0 channel
    DMA0REQbits.FORCE = 1; // Manual mode:Kick-start the 1st transfer
    BufferCount ^= 1;
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag} 
}

void UART1_Initialize(void) { // 115200 bps
    uint16_t i, *pS = U1Tx_PotVolt_mV;
    uint8_t defaultstring[8] = "0000 mV.";
    for (i = 0; i<sizeof (defaultstring); i++) {
        *pS++ = defaultstring[i];
        Nop();
    }
    pU1TxAddress = U1Tx_SimMsg;
    U1MODE = 0x8008 & ~(1 << 15);
    U1STA = 0x0000;
    U1BRG = U1BRP_VAL;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    /* Wait at least 9 microseconds (1/115200) before sending first char */
    for (i = 0; i < (FCY / U1BAUDRATE + 20); i++) {
        Nop();
    }
#ifdef USING_SIMULATOR
    IEC0bits.U1TXIE = 1; // Enable UART TX interrupt
#else
    DMA0_Initialize();
#endif
}

void DMA0_Initialize(void) {
    DMA0CON = 0x2001; // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA0CNT = 7; // Eight DMA requests
    DMA0REQbits.IRQSEL = 0x000C; // Select UART1 transmitter
    DMA0PAD = (volatile uint16_t) & U1TXREG;
    DMA0STAL = __builtin_dmaoffset(U1Tx_PotVolt_mV);
    DMA0STAH = 0x0000;

    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
}

void DMA1_Initialize(void) {
    DMA1CON = 0x0002; // Continuous, Ping-Pong, Post-Inc., Periph-RAM
    DMA1CNT = 7; // Eight DMA requests
    DMA1REQ = 0x000B; // Select UART1 receiver
    DMA1PAD = (volatile uint16_t) & U1RXREG;
    DMA1STAL = __builtin_dmaoffset(U1Rx_BufferA);
    DMA1STAH = 0x0000;
    DMA1STBL = __builtin_dmaoffset(U1Rx_BufferB);
    DMA1STBH = 0x0000;
    IFS0bits.DMA1IF = 0; // Clear DMA interrupt
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt
    DMA1CONbits.CHEN = 1; // Enable DMA channel
}
