/*
 * File:   LCM.c
 * Author: user
 *
 * Created on April 30, 2020, 12:34 PM
 */

#include "Common.h"
#include "LCM.h"

#define LCM_RS LATGbits.LATG15
#define LCM_RW LATEbits.LATE6
#define LCM_E LATEbits.LATE3

#define LCM_D4 LATCbits.LATC3
#define LCM_D5 LATCbits.LATC4
#define LCM_D6 LATCbits.LATC1
#define LCM_D7 LATCbits.LATC2

#define DIR_LCM_RS TRISGbits.TRISG15
#define DIR_LCM_RW TRISEbits.TRISE6
#define DIR_LCM_E TRISEbits.TRISE3

#define DIR_LCM_D4 TRISCbits.TRISC3
#define DIR_LCM_D5 TRISCbits.TRISC4
#define DIR_LCM_D6 TRISCbits.TRISC1
#define DIR_LCM_D7 TRISCbits.TRISC2

void LCM_IOSetup(void) {
    DIR_LCM_RS = 0;
    DIR_LCM_RW = 0;
    DIR_LCM_E = 0;

    DIR_LCM_D4 = 0;
    DIR_LCM_D5 = 0;
    DIR_LCM_D6 = 0;
    DIR_LCM_D7 = 0;

    LCM_RS = 0;
    LCM_RW = 0;
    LCM_E = 0;

    LCM_D4 = 0;
    LCM_D5 = 0;
    LCM_D6 = 0;
    LCM_D7 = 0;
}

void IntLCM_Communication(uint8_t NibbleByte) {
    uint8_t Temporary;
    //Temporary = NibbleByte & 0x0f;
    Delay_us(1); // tAS

    LCM_D7 = 0;
    LCM_D6 = 0;
    LCM_D5 = 0;
    LCM_D4 = 0;

    Temporary = NibbleByte & 0x0f;
    if ((Temporary & 0x08) == 0x08) // D7
        LCM_D7 = 1;

    Temporary = NibbleByte & 0x0f;
    if ((Temporary & 0x04) == 0x04) // D6
        LCM_D6 = 1;

    Temporary = NibbleByte & 0x0f;
    if ((Temporary & 0x02) == 0x02) // D5
        LCM_D5 = 1;

    Temporary = NibbleByte & 0x0f;
    if ((Temporary & 0x01) == 0x01) // D4
        LCM_D4 = 1;

    LCM_E = 1;
    Delay_us(1); //PWEH

    LCM_E = 0;
    Delay_us(1); //tAH
}

uint8_t LCM_IsBusy(void) {

    /*
        uint8_t Temporary;

        PMPSetAddress( LCM_INSTRUCTION );
        Temporary = mPMPMasterReadByte( );		// Dummy Read for trigger PMP active.

        while( !INTGetFlag( INT_PMP ) );		// Wait PMP Complete.
        INTClearFlag( INT_PMP );

        Temporary = mPMPMasterReadByte( );

        return ( ( Temporary >> 7 ) & 0x01 ); 
     */

    uint16_t i;

    for (i = 0; i < 500; i++);
    return 0;
}

void IntLCM_WriteData(uint8_t Data) {
#ifndef USING_SIMULATOR
    while (LCM_IsBusy());
#endif
    LCM_RS = 1; // RS = 1 , Data
    LCM_RW = 0;

    IntLCM_Communication((Data >> 4) & 0x0f);
    IntLCM_Communication(Data & 0x0f);
}

void IntLCM_WriteInstruction(uint8_t Instruction) {
#ifndef USING_SIMULATOR
    while (LCM_IsBusy());
#endif
    LCM_RS = 0; // RS = 0 , Instruction
    LCM_RW = 0;

    IntLCM_Communication((Instruction >> 4) & 0x0f);
    IntLCM_Communication(Instruction & 0x0f);
}

void LCMInitialize(void) {
    LCM_IOSetup();

    LCM_RS = 0; // RS = 0 , Instruction
    LCM_RW = 0;

    IntLCM_Communication(0x03); // Reset
    Delay_us(4100); // Wait 4.1 mSecs

    IntLCM_Communication(0x03); // Reset
    Delay_us(100); // Wait 100 uSecs

    IntLCM_Communication(0x03); // Reset
    Delay_us(100); // Wait 100 uSecs

    IntLCM_Communication(0x02);
    Delay_us(100); // Wait 100 uSecs

    IntLCM_Communication((0x28 >> 4) & 0x0f);
    IntLCM_Communication(0x28 & 0xf);
    Delay_us(37);

    IntLCM_Communication((Disp_Off >> 4) & 0x0f); // Dsplay Off
    IntLCM_Communication(Disp_Off & 0xf);
    Delay_us(37);

    IntLCM_Communication((Disp_Clear >> 4) & 0x0f); // Dsplay Clear
    IntLCM_Communication(Disp_Clear & 0xf);
    Delay_us(1520);

    IntLCM_Communication((Disp_Entry_Inc >> 4) & 0x0f); // Entry Mode
    IntLCM_Communication(Disp_Entry_Inc & 0xf);
    Delay_us(37);

    IntLCM_Communication((Disp_On >> 4) & 0x0f); // Display On
    IntLCM_Communication(Disp_On & 0xf);
    Delay_us(37);
}

void LCM_SetCursor(uint8_t Y, uint8_t X) {
    IntLCM_WriteInstruction((0x80 + (Y * 0x40) + X));
}

void LCM_PutASCII(uint8_t Ascii) {
    IntLCM_WriteData(Ascii);
}

void LCM_PutHex(uint8_t Hex) {
    uint8_t Temporary;

    Temporary = (Hex >> 4) & 0x0f;

    if (Temporary > 9)
        Temporary += 0x37;
    else
        Temporary += '0';

    IntLCM_WriteData(Temporary);

    Temporary = Hex & 0x0f;
    if (Temporary > 9)
        Temporary += 0x37;
    else Temporary += '0';
    IntLCM_WriteData(Temporary);
}

void LCM_PutROMString(const uint8_t *String) {
    while (*String != 0x00)
        IntLCM_WriteData(*String++);
}

void LCM_PutRAMString(uint8_t *String) {
    while (*String != 0x00)
        IntLCM_WriteData(*String++);
}

static uint8_t Disable_Zero = 1;

uint8_t LCM_BCD_Regulate(uint8_t BCD) {
    if (BCD == 0) {
        if (Disable_Zero == 1)
            return ' ';
        else
            return '0';
    } else {
        Disable_Zero = 0;
        return ( BCD += '0');
    }
}

void LCM_PutNumber(uint16_t Number, uint8_t Digit) {
    uint8_t Temporary;
    Disable_Zero = 1;

    switch (Digit) {
        default:
        case 4:
            Temporary = LCM_BCD_Regulate((Number % 10000) / 1000);
            IntLCM_WriteData(Temporary);

        case 3:
            Temporary = LCM_BCD_Regulate((Number % 1000) / 100);
            IntLCM_WriteData(Temporary);

        case 2:
            Temporary = LCM_BCD_Regulate((Number % 100) / 10);
            IntLCM_WriteData(Temporary);

        case 1:
            Disable_Zero = 0;
            Temporary = LCM_BCD_Regulate(Number % 10);
            IntLCM_WriteData(Temporary);
    }
}
