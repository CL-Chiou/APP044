/*
 * File:   adc.c
 * Author: user
 *
 * Created on May 5, 2020, 9:40 AM
 */

#include <xc.h>
#include "Common.h"
#include "Timers.h"
#include "adc1.h"

__eds__ uint16_t ADC1BUF[8] __attribute__((eds, space(dma)));
uint16_t ADC1CH0123_mV[4];
uint16_t VR1_8bit;

void (*ADC1_AD1InterruptHandler)(void);
void (*ADC1_DMA2InterruptHandler)(void);
static void AD1CallBack(void);
static void DMA2CallBack(void);

void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void) {
    DMA2CallBack();
    IFS1bits.DMA2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _AD1Interrupt(void) {
    AD1CallBack();
    IFS0bits.AD1IF = 0;
}

void ADC1Initialize(void) { /*TAD >= 117.6nS*/
    /* Set port configuration */
    ANSELBbits.ANSB0 = 1; // Ensure AN0/RB0 is analog; VR1_8bit
    ANSELBbits.ANSB1 = 1; // Ensure AN1/RB1 is analog; ANIN4
    ANSELBbits.ANSB2 = 1; // Ensure AN2/RB2 is analog; ANIN3
    ANSELBbits.ANSB3 = 1; // Ensure AN3/RB3 is analog; ANIN2
    ANSELBbits.ANSB4 = 1; // Ensure AN4/RB4 is analog; ANIN1
    /* Initialize and enable ADC module */
    AD1CON1bits.ADSIDL = 0; //0 = Continues module operation in Idle mode
    AD1CON1bits.ADDMABM = 1; //1 = DMA buffers are written in the order of conversion. The module provides an address to the DMA
    //channel that is the same as the address used for the non-DMA stand-alone buffer
    AD1CON1bits.AD12B = 0; //0 = 10-bit, 4-channel ADC operation
    AD1CON1bits.FORM = 0b00; //00 = Integer (DOUT = 0000 dddd dddd dddd)
    AD1CON1bits.SSRC = 0b010; //010 = Timer3 compare ends sampling and starts conversion
    AD1CON1bits.SSRCG = 0;
    AD1CON1bits.SIMSAM = 1; //1 = Samples CH0, CH1, CH2, CH3 simultaneously 
    AD1CON1bits.ASAM = 1; //1 = Sampling begins immediately after the last conversion; SAMP bit is auto-set

    AD1CON2bits.VCFG = 0b000; //000 VREFH:AVDD ; VREFL: AVSS
    AD1CON2bits.CSCNA = 0; //0 = Do not scan inputs
    AD1CON2bits.BUFM = 0; //0 = Always starts filling the buffer from the Start address.
    AD1CON2bits.BUFS = 0;
    AD1CON2bits.SMPI = 0b00011; //00011 = Generates interrupt after completion of every 4th sample/conversion operation
    AD1CON2bits.CHPS = 0b10; //1x = Converts CH0, CH1, CH2 and CH3

    AD1CON3bits.ADRC = 0; //0 = Clock Derived From System Clock
    AD1CON3bits.SAMC = 0xF; // 15TAD
    AD1CON3bits.ADCS = 15; //TP • (ADCS<7:0> + 1) = 16 • TCY

#ifdef USING_SIMULATOR
    AD1CON4bits.ADDMAEN = 0; //0 = Conversion results are stored in ADCxBUF0 through ADCxBUFF registers; DMA will not be used
#else
    AD1CON4bits.ADDMAEN = 1; //1 = Conversion results are stored in ADCxBUF0 register for transferring to RAM using DMA
#endif
    AD1CON4bits.DMABL = 0b011; //011 = Allocates 8 words of buffer to each analog input

    /* Assign MUXA inputs */
    AD1CHS123bits.CH123SA = 0;
    AD1CHS123bits.CH123NA = 0;
    AD1CHS0bits.CH0SA = 3; // 00011 = Channel 0 positive input is AN3
    AD1CHS0bits.CH0NA = 0; // 0 = Channel 0 negative input is VREFL
    //AD1CSSH/AD1CSSL:Analog-to-Digital Input Scan Selection Register
    AD1CSSH = 0x0000;
    AD1CSSL = 0x0000;
#ifdef USING_SIMULATOR
    IEC0bits.AD1IE = 1;
    ADC1_AD1SetIntHandler(ADC1_AD1DefInterruptHandler);
#endif
    IPC3bits.AD1IP = 1;
    AD1CON1bits.ADON = 1;

    Delay_us(20);
    Timer3_Initialize();
#ifndef USING_SIMULATOR
    DMA2_Initialize();
#endif
}

void DMA2_Initialize(void) {
    DMA2CON = 0x0000; // Continuous, Post-Increment, Peripheral-to-RAM
    DMA2CNT = 3; // Four DMA requests
    DMA2REQbits.IRQSEL = 0x000D; // Select ADC1 convert done
    DMA2PAD = (volatile uint16_t) & ADC1BUF0;
    DMA2STAL = __builtin_dmaoffset(ADC1BUF);
    DMA2STAH = 0x0000;
    IFS1bits.DMA2IF = 0; // Clear DMA Interrupt Flag
    IEC1bits.DMA2IE = 1; // Enable DMA interrupt
    DMA2CONbits.CHEN = 1; //Enable DMA channel
    ADC1_DMA2SetIntHandler(ADC1_DMA2DefInterruptHandler);
}

void ADC1_AD1SetIntHandler(void *handler) {
    ADC1_AD1InterruptHandler = handler;
}

void ADC1_DMA2SetIntHandler(void *handler) {
    ADC1_DMA2InterruptHandler = handler;
}

static void AD1CallBack(void) {
    if (ADC1_AD1InterruptHandler) {
        ADC1_AD1InterruptHandler();
    }
}

static void DMA2CallBack(void) {
    if (ADC1_DMA2InterruptHandler) {
        ADC1_DMA2InterruptHandler();
    }
}