/*
 * File:   Timers.c
 * Author: user
 *
 * Created on 2020年4月29日, 下午 4:46
 */


#include <xc.h>
#include "Common.h"

extern struct _flag FLAG;
extern struct _clock CLOCK;

unsigned int T1Counter, T1Counter_1ms, T1Counter_1000ms;
/*ADC*/
extern int VR1;
extern int ADCValues[8];
/*USB CDC*/
extern char CDC_Buffer[64];
/*Buzzer*/
extern unsigned char BuzzerTimeCNT;
extern unsigned char BuzzerFlag;

void CAN_Send_VR1_Message(void);

/*code for Timer1 ISR*/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    /* Interrupt Service Routine code goes here */
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
    T1Counter++;
    T1Counter_1ms++;
    BuzzerTimeCNT++;
    if (T1Counter >= 10000) {
        T1Counter = 0;
        T1Counter_1000ms++;
        LED_Toggle(LED_D1);
        if (T1Counter_1000ms >= 2) {
            T1Counter_1000ms = 0;
            FLAG.Second = 1;
        }
    }
    if (BuzzerTimeCNT <= 4 && BuzzerFlag) Buzzer = 1;
    else Buzzer = 0;
    if (BuzzerTimeCNT >= 8) BuzzerTimeCNT = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
}

void Timer1_Initial(void) { //20kHz
    T1CONbits.TCKPS = 0b00; //00 = 1:1
    T1CONbits.TCS = 0; //0 = Internal clock (FP)
    TMR1 = 0x00; // Clear timer register
    PR1 = 2000; // 0.05 ms interrupt interval @ FOSC=80Mhz,FP=40MHz Prescale 1:1
    IPC0bits.T1IP = 0x01; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    T1CONbits.TON = 1; //1 = Starts 16-bit Timer1

}

void Timer3_Initial(void) { //1kHz
    T3CONbits.TCKPS = 0b01; //01 = 1:8
    T3CONbits.TCS = 0;
    TMR3 = 0x00;
    PR3 = 5000;
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 0;
    T3CONbits.TON = 1;
}
