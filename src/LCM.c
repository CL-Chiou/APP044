/*
 * File:   LCD1602.c
 * Author: user
 *
 * Created on 2020年4月30日, 下午 12:34
 */


#include <xc.h>
#include "Common.h"

#define IntLCM_Delay1us         ( FCY * 1 ) / 1000000L / 60
#define IntLCM_Delay10us        ( FCY * 10 ) / 1000000L / 60
#define IntLCM_Delay100us       ( FCY * 100 ) / 1000000L / 60

#define IntLCM_Delay1ms			( FCY * 1 ) / 1000L / 60
#define IntLCM_Delay10ms		( FCY * 10 ) / 1000L / 60
#define IntLCM_Delay100ms		( FCY * 100 ) / 1000L / 60

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

void LCM_Delay(unsigned long Count) {
    unsigned char i = 0;
    unsigned long j = 0;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < Count; j++);
    }
}

void IntLCM_Communication(unsigned char NibbleByte) {
    unsigned char Temporary;
    //Temporary = NibbleByte & 0x0f;

    LCM_Delay(IntLCM_Delay1us); // tAS

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
    LCM_Delay(IntLCM_Delay1us); //PWEH

    LCM_E = 0;
    LCM_Delay(IntLCM_Delay1us); //tAH
}

unsigned char LCM_IsBusy(void) {

    /*
        unsigned char Temporary;

        PMPSetAddress( LCM_INSTRUCTION );
        Temporary = mPMPMasterReadByte( );		// Dummy Read for trigger PMP active.

        while( !INTGetFlag( INT_PMP ) );		// Wait PMP Complete.
        INTClearFlag( INT_PMP );

        Temporary = mPMPMasterReadByte( );

        return ( ( Temporary >> 7 ) & 0x01 ); 
     */

    unsigned int i;

    for (i = 0; i < 500; i++);
    return 0;
}

void IntLCM_WriteData(unsigned char Data) {
    while (LCM_IsBusy());
    LCM_RS = 1; // RS = 1 , Data
    LCM_RW = 0;

    IntLCM_Communication((Data >> 4) & 0x0f);
    IntLCM_Communication(Data & 0x0f);
}

void IntLCM_WriteInstruction(unsigned char Instruction) {
    while (LCM_IsBusy());
    LCM_RS = 0; // RS = 0 , Instruction
    LCM_RW = 0;

    IntLCM_Communication((Instruction >> 4) & 0x0f);
    IntLCM_Communication(Instruction & 0x0f);
}

void LCM_Init(void) {
    LCM_IOSetup();

    LCM_RS = 0; // RS = 0 , Instruction
    LCM_RW = 0;

    IntLCM_Communication(0x03); // Reset
    LCM_Delay(IntLCM_Delay1ms * 4.1); // Wait 4.1 mSecs

    IntLCM_Communication(0x03); // Reset
    LCM_Delay(IntLCM_Delay100us); // Wait 100 uSecs

    IntLCM_Communication(0x03); // Reset
    LCM_Delay(IntLCM_Delay100us); // Wait 100 uSecs

    IntLCM_Communication(0x02);
    LCM_Delay(IntLCM_Delay100us); // Wait 100 uSecs

    IntLCM_Communication((0x28 >> 4) & 0x0f);
    IntLCM_Communication(0x28 & 0xf);
    LCM_Delay(IntLCM_Delay10us * 3.7);

    IntLCM_Communication((Disp_Off >> 4) & 0x0f); // Dsplay Off
    IntLCM_Communication(Disp_Off & 0xf);
    LCM_Delay(IntLCM_Delay10us * 3.7);

    IntLCM_Communication((Disp_Clear >> 4) & 0x0f); // Dsplay Clear
    IntLCM_Communication(Disp_Clear & 0xf);
    LCM_Delay(IntLCM_Delay1ms * 1.52);

    IntLCM_Communication((Disp_Entry_Inc >> 4) & 0x0f); // Entry Mode
    IntLCM_Communication(Disp_Entry_Inc & 0xf);
    LCM_Delay(IntLCM_Delay10us * 3.7);

    IntLCM_Communication((Disp_On >> 4) & 0x0f); // Display On
    IntLCM_Communication(Disp_On & 0xf);
    LCM_Delay(IntLCM_Delay10us * 3.7);
}

void LCM_SetCursor(unsigned char Y, unsigned char X) {
    IntLCM_WriteInstruction((0x80 + (Y * 0x40) + X));
}

void LCM_PutASCII(unsigned char Ascii) {
    IntLCM_WriteData(Ascii);
}

void LCM_PutHex(unsigned char Hex) {
    unsigned char Temporary;

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

void LCM_PutROMString(const unsigned char *String) {
    while (*String != 0x00)
        IntLCM_WriteData(*String++);
}

void LCM_PutRAMString(unsigned char *String) {
    while (*String != 0x00)
        IntLCM_WriteData(*String++);
}

static unsigned char Disable_Zero = 1;

unsigned char LCM_BCD_Regulate(unsigned char BCD) {
    if (BCD == 0) {
        if (Disable_Zero == 1)
            return ' ';
        else
            return '0';
    }
    else {
        Disable_Zero = 0;
        return ( BCD += '0');
    }
}

void LCM_PutNumber(unsigned int Number, unsigned char Digit) {
    unsigned char Temporary;
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
