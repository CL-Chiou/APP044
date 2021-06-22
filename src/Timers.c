/*
 * File:   Timers.c
 * Author: user
 *
 * Created on 2020年4月29日, 下午 4:46
 */


#include <xc.h>
#include "Common.h"

extern TimeFlag_t FLAG;
extern RealTimeClock_t CLOCK;

uint16_t T1Cnt, T1Cnt_1ms, T1Cnt_1000ms;
uint8_t T1Cnt_RLED_Period;
extern uint16_t TCnt_50ms;

/*LED*/
extern uint8_t LED2Blink, LED2BlinkDuty;
/*ADC*/
extern int ADCValues[8];
/*USB CDC*/
extern uint8_t CDCSendBuffer[64];
/*Buzzer*/
extern uint8_t BzTCnt;
extern uint8_t BzOutput;
/*RTC*/
extern uint8_t rtccSecondChanged, rtccReadFailure;

void CAN_Send_VR1_Message(void);

/*code for Timer1 ISR*/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    /* Interrupt Service Routine code goes here */
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
    T1Cnt++;
    T1Cnt_1ms++;
    T1Cnt_RLED_Period++;
    BzTCnt++;
    if (T1Cnt >= 10000) {
        T1Cnt = 0;
        T1Cnt_1000ms++;
        if (T1Cnt_1000ms >= 2) {
            T1Cnt_1000ms = 0;
            LED2Blink = 1;
        }
    }
    if (rtccSecondChanged == 1 || rtccReadFailure == 1) {
        static uint16_t rtccSecondChangedCnt = 0;
        if (rtccSecondChangedCnt >= 20000) {
            FLAG.Second = 1;
            rtccSecondChangedCnt = 0;
        } else {
            rtccSecondChangedCnt++;
        }
    }
    if (T1Cnt_RLED_Period >= 100) T1Cnt_RLED_Period = 0;
    if (T1Cnt_RLED_Period <= LED2BlinkDuty) {
        LEDTurnOn(LED_D2);
    } else {
        LEDTurnOff(LED_D2);
    }
    if (BzTCnt <= 4 && BzOutput) Buzzer = 1;
    else Buzzer = 0;
    if (BzTCnt >= 8) BzTCnt = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
    if (++TCnt_50ms >= 50) {
        TCnt_50ms = 0x00;
        DMA0CONbits.CHEN = 1; // Enable DMA0 channel
        DMA0REQbits.FORCE = 1; // Manual mode:Kick-start the 1st transfer
    }
}

void Timer1_Initialize(void) { //20kHz
    T1CONbits.TCKPS = 0b00; //00 = 1:1
    T1CONbits.TCS = 0; //0 = Internal clock (FP)
    TMR1 = 0x00; // Clear timer register
    PR1 = 2000; // 0.05 ms interrupt interval @ FOSC=80Mhz,FP=40MHz Prescale 1:1
    IPC0bits.T1IP = 0x01; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    T1CONbits.TON = 1; //1 = Starts 16-bit Timer1
}

void Timer3_Initialize(void) {
#ifdef USING_SIMULATOR 
    /*2.5Hz*/
    T3CONbits.TCKPS = 0b11; //01 = 1:256
    T3CONbits.TCS = 0;
    TMR3 = 0x00;
    PR3 = 62500;
#else
    /*1kHz*/
    T3CONbits.TCKPS = 0b01; //01 = 1:8
    T3CONbits.TCS = 0;
    TMR3 = 0x00;
    PR3 = 5000;
#endif
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}
