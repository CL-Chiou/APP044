#include <xc.h>
#include "Common.h"

#define     SEGMENT_1A   LATAbits.LATA7 
#define     SEGMENT_1B   LATGbits.LATG9
#define     SEGMENT_1C   LATGbits.LATG12
#define     SEGMENT_1D   LATEbits.LATE1
#define     SEGMENT_1E   LATGbits.LATG14
#define     SEGMENT_1F   LATEbits.LATE0 
#define     SEGMENT_1G   LATGbits.LATG0
#define     SEGMENT_1DOT LATAbits.LATA6 

#define     SEGMENT_2A   LATDbits.LATD1
#define     SEGMENT_2B   LATDbits.LATD4
#define     SEGMENT_2C   LATDbits.LATD13
#define     SEGMENT_2D   LATDbits.LATD3
#define     SEGMENT_2E   LATDbits.LATD12
#define     SEGMENT_2F   LATDbits.LATD2
#define     SEGMENT_2G   LATCbits.LATC13
#define     SEGMENT_2DOT LATCbits.LATC14

#define     COMMON_1        LATDbits.LATD5
#define     COMMON_2       LATGbits.LATG1
#define     COMMON_3        LATDbits.LATD6
#define     COMMON_4        LATFbits.LATF1
#define     COMMON_5        LATDbits.LATD7
#define     COMMON_6        LATFbits.LATF0


#define     DIR_COMMON_1    TRISDbits.TRISD5 
#define     DIR_COMMON_2    TRISGbits.TRISG1
#define     DIR_COMMON_3    TRISDbits.TRISD6
#define     DIR_COMMON_4    TRISFbits.TRISF1
#define     DIR_COMMON_5    TRISDbits.TRISD7
#define     DIR_COMMON_6    TRISFbits.TRISF0 


#define     DIR_SEGMENT_1A   TRISAbits.TRISA7
#define     DIR_SEGMENT_1B  TRISGbits.TRISG9
#define     DIR_SEGMENT_1C  TRISGbits.TRISG12
#define     DIR_SEGMENT_1D  TRISEbits.TRISE1
#define     DIR_SEGMENT_1E  TRISGbits.TRISG14
#define     DIR_SEGMENT_1F  TRISEbits.TRISE0
#define     DIR_SEGMENT_1G  TRISGbits.TRISG0
#define     DIR_SEGMENT_1DOT    TRISAbits.TRISA6

#define     DIR_SEGMENT_2A   TRISDbits.TRISD1
#define     DIR_SEGMENT_2B  TRISDbits.TRISD4
#define     DIR_SEGMENT_2C  TRISDbits.TRISD13
#define     DIR_SEGMENT_2D  TRISDbits.TRISD3
#define     DIR_SEGMENT_2E  TRISDbits.TRISD12
#define     DIR_SEGMENT_2F  TRISDbits.TRISD2
#define     DIR_SEGMENT_2G  TRISCbits.TRISC13
#define     DIR_SEGMENT_2DOT    TRISCbits.TRISC14


uint8_t FirstLineData[6];
uint8_t SecondLineData[6];
uint16_t ScanDigit = 1;
const uint8_t SevenSegPattern[16] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00};

volatile uint16_t Scan_Index = 0;

union SEGMENT_Template {

    struct {
        unsigned SEG_A : 1;
        unsigned SEG_B : 1;
        unsigned SEG_C : 1;
        unsigned SEG_D : 1;
        unsigned SEG_E : 1;
        unsigned SEG_F : 1;
        unsigned SEG_G : 1;
        unsigned SEG_DOT : 1;
    };
    uint8_t Byte_Val;
} SEGMENT_Data;

void LINE_12_Initialize(void) {
    DIR_SEGMENT_1A = 0;
    DIR_SEGMENT_1B = 0;
    DIR_SEGMENT_1C = 0;
    DIR_SEGMENT_1D = 0;
    DIR_SEGMENT_1E = 0;
    DIR_SEGMENT_1F = 0;
    DIR_SEGMENT_1G = 0;
    DIR_SEGMENT_1DOT = 0;

    DIR_SEGMENT_2A = 0;
    DIR_SEGMENT_2B = 0;
    DIR_SEGMENT_2C = 0;
    DIR_SEGMENT_2D = 0;
    DIR_SEGMENT_2E = 0;
    DIR_SEGMENT_2F = 0;
    DIR_SEGMENT_2G = 0;
    DIR_SEGMENT_2DOT = 0;


    DIR_COMMON_1 = 0;
    DIR_COMMON_2 = 0;
    DIR_COMMON_3 = 0;
    DIR_COMMON_4 = 0;
    DIR_COMMON_5 = 0;
    DIR_COMMON_6 = 0;

    LINE_12_Write_Default();
}

void Scan7Segment(void) {
    COMMON_1 = 0;
    COMMON_2 = 0;
    COMMON_3 = 0;
    COMMON_4 = 0;
    COMMON_5 = 0;
    COMMON_6 = 0;

    SEGMENT_Data.Byte_Val = FirstLineData[Scan_Index];
    if (SEGMENT_Data.SEG_A) SEGMENT_1A = 1;
    else SEGMENT_1A = 0;
    if (SEGMENT_Data.SEG_B) SEGMENT_1B = 1;
    else SEGMENT_1B = 0;

    if (SEGMENT_Data.SEG_C) SEGMENT_1C = 1;
    else SEGMENT_1C = 0;
    if (SEGMENT_Data.SEG_D) SEGMENT_1D = 1;
    else SEGMENT_1D = 0;

    if (SEGMENT_Data.SEG_E) SEGMENT_1E = 1;
    else SEGMENT_1E = 0;
    if (SEGMENT_Data.SEG_F) SEGMENT_1F = 1;
    else SEGMENT_1F = 0;

    if (SEGMENT_Data.SEG_G) SEGMENT_1G = 1;
    else SEGMENT_1G = 0;
    if (SEGMENT_Data.SEG_DOT) SEGMENT_1DOT = 1;
    else SEGMENT_1DOT = 0;

    //  *******************************
    //  Write scan data for 2nd line
    //  *******************************

    SEGMENT_Data.Byte_Val = SecondLineData[Scan_Index];
    if (SEGMENT_Data.SEG_A) SEGMENT_2A = 1;
    else SEGMENT_2A = 0;
    if (SEGMENT_Data.SEG_B) SEGMENT_2B = 1;
    else SEGMENT_2B = 0;

    if (SEGMENT_Data.SEG_C) SEGMENT_2C = 1;
    else SEGMENT_2C = 0;
    if (SEGMENT_Data.SEG_D) SEGMENT_2D = 1;
    else SEGMENT_2D = 0;

    if (SEGMENT_Data.SEG_E) SEGMENT_2E = 1;
    else SEGMENT_2E = 0;
    if (SEGMENT_Data.SEG_F) SEGMENT_2F = 1;
    else SEGMENT_2F = 0;

    if (SEGMENT_Data.SEG_G) SEGMENT_2G = 1;
    else SEGMENT_2G = 0;
    if (SEGMENT_Data.SEG_DOT) SEGMENT_2DOT = 1;
    else SEGMENT_1DOT = 0;

    switch (Scan_Index) {
        case 0: COMMON_1 = 1;
            break;
        case 1: COMMON_2 = 1;
            break;
        case 2: COMMON_3 = 1;
            break;
        case 3: COMMON_4 = 1;
            break;
        case 4: COMMON_5 = 1;
            break;
        case 5: COMMON_6 = 1;
            break;
        default:
            break;
    }

    Scan_Index += 1;
    if (Scan_Index >= 6) Scan_Index = 0;

}

void LINE_12_Write_Default(void) {
    FirstLineData[0] = SevenSegPattern[10];
    FirstLineData[1] = SevenSegPattern[10];
    FirstLineData[2] = SevenSegPattern[10];
    FirstLineData[3] = SevenSegPattern[10];
    FirstLineData[4] = SevenSegPattern[10];
    FirstLineData[5] = SevenSegPattern[10];

    SecondLineData[0] = SevenSegPattern[10];
    SecondLineData[1] = SevenSegPattern[10];
    SecondLineData[2] = SevenSegPattern[10];
    SecondLineData[3] = SevenSegPattern[10];
    SecondLineData[4] = SevenSegPattern[10];
    SecondLineData[5] = SevenSegPattern[10];
}
