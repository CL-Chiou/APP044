/*
 * File:   pwm.c
 * Author: user
 *
 * Created on November 23, 2020, 10:23 AM
 */

#include "pwm.h"

#include "Common.h"

void PWMInitialize(void) {
    /* Set PWM Period on Primary Time Base */ /*FPWM = FOSC / (PTPER * PWM Input Clock Prescaler); 3200 = 80,000,000 /
                                                 (25k(Hz)*1)*/
    PTPER = 3200;

    /* Set Phase Shift */
    PHASE3  = 0;
    SPHASE3 = 0;

    PHASE5 = 3200;
    PDC5   = 500;

    PHASE6  = 3200;
    SPHASE6 = 6400;
    /* Set Duty Cycles */
    MDC = 0;

    /* Set Dead Time Values */
    DTR3    = 0;
    ALTDTR3 = 0;

    DTR5    = 0;
    ALTDTR5 = 300;

    DTR6    = 25;
    ALTDTR6 = 35;

    /* Set PWM Mode to Independent */
    IOCON3bits.PENH = 1;    /*1 = PWM module controls PWMxH pin*/
    IOCON3bits.PENL = 0;    /*0 = GPIO module controls PWMxL pin*/
    IOCON3bits.PMOD = 0b11; /*11 = PWM I/O pin pair is in True Independent PWM Output mode(3)*/

    IOCON5bits.PENH = 1;    /*1 = PWM module controls PWMxH pin*/
    IOCON5bits.PENL = 1;    /*1 = PWM module controls PWMxL pin*/
    IOCON5bits.PMOD = 0b00; /*00 = PWM I/O pin pair is in Complementary Output mode*/

    IOCON6bits.PENH = 1;    /*1 = PWM module controls PWMxH pin*/
    IOCON6bits.PENL = 1;    /*1 = PWM module controls PWMxL pin*/
    IOCON6bits.PMOD = 0b11; /*11 = PWM I/O pin pair is in True Independent PWM Output mode(3)*/

    /* Set Independent Time Bases, Edge-Aligned Mode and Master Duty Cycles */
    PWMCON3bits.MDCS = 1; /*1 = MDC register provides duty cycle information for this PWM generator*/

    PWMCON5bits.MDCS = 0; /*0 = PDCx and SDCx registers provide duty cycle information for this PWM generator*/
    PWMCON5bits.CAM  = 1; /*1 = Center-Aligned mode is enabled*/
    PWMCON5bits.ITB  = 1; /*1 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/

    PWMCON6bits.MDCS = 1; /*1 = MDC register provides duty cycle information for this PWM generator*/
    PWMCON6bits.ITB  = 1;

    /* Configure Faults */
    FCLCON3bits.FLTMOD = 0b11; /*11 = Fault input is disabled*/
    FCLCON5bits.FLTMOD = 0b11; /*11 = Fault input is disabled*/
    FCLCON6bits.FLTMOD = 0b11; /*11 = Fault input is disabled*/

    /* 1:1 Prescaler */
    PTCON2bits.PCLKDIV = 0b000;

    /* Enable PWM Module */
    PTCONbits.PTEN = 1;
}
