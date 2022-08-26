/*
 * File:   Timers.c
 * Author: user
 *
 * Created on April 29, 2020, 4:46 PM
 */

#include <xc.h>

#include "Common.h"

uint16_t T1Cnt, T1Cnt_1ms, T1Cnt_1000ms;
uint8_t  T1Cnt_RLED_Period, T1Cnt_OLED_Period;

void (*Timer1_InterruptHandler)(void);
void (*Timer3_InterruptHandler)(void);
static void Timer1CallBack(void);
static void Timer3CallBack(void);

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    Timer1CallBack();
    IFS0bits.T1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    Timer3CallBack();
    IFS0bits.T3IF = 0;
}

void Timer1Initialize(void) {  // 20kHz
    T1CONbits.TCKPS = 0b00;    // 00 = 1:1
    T1CONbits.TCS   = 0;       // 0 = Internal clock (FP)
    TMR1            = 0x00;    // Clear timer register
    PR1             = 2000;    // 0.05 ms interrupt interval @ FOSC=80Mhz,FP=40MHz Prescale 1:1
    IPC0bits.T1IP   = 0x01;    // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF   = 0;       // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE   = 1;       // Enable Timer1 interrupt
    T1CONbits.TON   = 1;       // 1 = Starts 16-bit Timer1
    Timer1_SetIntHandler(Timer1_DefInterruptHandler);
}

void Timer3_Initialize(void) {
#ifdef USING_SIMULATOR
    /* 2.5Hz */
    T3CONbits.TCKPS = 0b11;  // 01 = 1:256
    T3CONbits.TCS   = 0;
    TMR3            = 0x00;
    PR3             = 62500;
#else
    /* 1kHz */
    T3CONbits.TCKPS = 0b01;  // 01 = 1:8
    T3CONbits.TCS   = 0;
    TMR3            = 0x00;
    PR3             = 5000;
#endif
    IPC2bits.T3IP = 0x01;  // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0;     // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
    Timer3_SetIntHandler(Timer3_DefInterruptHandler);
}

void Timer1_SetIntHandler(void *handler) {
    Timer1_InterruptHandler = handler;
}

void Timer3_SetIntHandler(void *handler) {
    Timer3_InterruptHandler = handler;
}

static void Timer3CallBack() {
    if (Timer3_InterruptHandler) {
        Timer3_InterruptHandler();
    }
}

static void Timer1CallBack() {
    if (Timer1_InterruptHandler) {
        Timer1_InterruptHandler();
    }
}
