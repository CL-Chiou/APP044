#include <xc.h>

/* Prototype function declaration for functions in the file */
void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void);
void __attribute__((interrupt, no_auto_psv)) _AddressError(void);
void __attribute__((interrupt, no_auto_psv)) _HardTrapError(void);
void __attribute__((interrupt, no_auto_psv)) _StackError(void);
void __attribute__((interrupt, no_auto_psv)) _MathError(void);
void __attribute__((interrupt, no_auto_psv)) _DMACError(void);
void __attribute__((interrupt, no_auto_psv)) _SoftTrapError(void);
void __attribute__((interrupt, no_auto_psv)) _ReservedTrap7(void);

/* Primary Exception Vector handlers:
These routines are used if INTCON2bits.ALTIVT = 0.
All trap service routines in this file simply ensure that device
continuously executes code within the trap service routine. Users
may modify the basic framework provided here to suit to the needs
of their application. */
//================================================================
// OSCFAIL: Oscillator Failure Trap Status bit

void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0; //Clear the trap flag
    while (1);
}
//================================================================
// ADDRERR: Address Error Trap Status bit
// DS read access when DSRPAG = 0x000 will force an Address Error trap.

void __attribute__((interrupt, no_auto_psv)) _AddressError(void) {
    INTCON1bits.ADDRERR = 0; //Clear the trap flag
    while (1);
}
//================================================================
// SGHT: Software Generated Hard Trap Status bit

void __attribute__((interrupt, no_auto_psv)) _HardTrapError(void) {
    INTCON4bits.SGHT = 0; //Clear the trap flag
    while (1);
}
//================================================================
// STKERR: Stack Error Trap Status bit

void __attribute__((interrupt, no_auto_psv)) _StackError(void) {
    INTCON1bits.STKERR = 0; //Clear the trap flag
    while (1);
}
//================================================================
// MATHERR: Math Error Status bit

void __attribute__((interrupt, no_auto_psv)) _MathError(void) {
    INTCON1bits.MATHERR = 0; //Clear the trap flag
    while (1);
}
//================================================================
// DMACERR: DMAC Trap Flag bit

void __attribute__((interrupt, no_auto_psv)) _DMACError(void) {
    INTCON1bits.DMACERR = 0; //Clear the trap flag
    while (1);
}
//================================================================
// SWTRAP: Software Trap Status bit

void __attribute__((interrupt, no_auto_psv)) _SoftTrapError(void) {
    INTCON2bits.SWTRAP = 0; //Clear the trap flag
    while (1);
}
//================================================================

void __attribute__((interrupt, no_auto_psv)) _ReservedTrap7(void) { // INTCON1bits.DMACERR = 0; //Clear the trap flag
    while (1);
}