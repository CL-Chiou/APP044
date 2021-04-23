/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

#ifndef MCP4922_H
#define MCP4922_H

#include "Common.h"

typedef enum {
    A,
    B
} A_B;

typedef enum {
    _2x, //0 = 2x (VOUT = 2 * VREF * D/4096)
    _1x //1 = 1x (VOUT = VREF * D/4096)
} GA;

typedef union {

    struct {
        unsigned Data11_0 : 12;
        unsigned nSHDN : 1;
        GA nGA : 1;
        unsigned BUF : 1;
        A_B nA_B : 1;
    };
    uint16_t Int;
} mcp4922cmd;

void MCP4922_2SineOutput(void);

#endif //MCP4922_H
