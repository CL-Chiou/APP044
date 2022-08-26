/*
 * File:   MCP4922.c
 * Author: user
 *
 * Created on April 23, 2021, 5:02 PM
 */

#include "MCP4922.h"

#include "Common.h"
#include "spi1.h"

mcp4922cmd DAC_A =
               {
                   .nA_B     = A,
                   .BUF      = 0,
                   .nGA      = _2x,  //= 2x (VOUT = 2 * VREF * D/4096)
                   .nSHDN    = 1,
                   .Data11_0 = 0,
},
           DAC_B = {
               .nA_B     = B,
               .BUF      = 0,
               .nGA      = _2x,  //= 2x (VOUT = 2 * VREF * D/4096)
               .nSHDN    = 1,
               .Data11_0 = 0,
};

uint8_t SPINumberOfTx;
uint8_t SineIndex;

const uint16_t Sine[2][120] = {
    {5446, 5517, 5587, 5657, 5727, 5795, 5863, 5930, 5995, 6059, 6121, 6181, 6240, 6296, 6349, 6401, 6449, 6495,
     6538, 6578, 6615, 6649, 6679, 6706, 6730, 6750, 6766, 6779, 6789, 6794, 6796, 6794, 6789, 6779, 6766, 6750,
     6730, 6706, 6679, 6649, 6615, 6578, 6538, 6495, 6449, 6401, 6349, 6296, 6240, 6181, 6121, 6059, 5995, 5930,
     5863, 5795, 5727, 5657, 5587, 5517, 5446, 5375, 5305, 5235, 5165, 5097, 5029, 4962, 4897, 4833, 4771, 4711,
     4652, 4596, 4543, 4491, 4443, 4397, 4354, 4314, 4277, 4243, 4213, 4186, 4162, 4142, 4126, 4113, 4103, 4098,
     4096, 4098, 4103, 4113, 4126, 4142, 4162, 4186, 4213, 4243, 4277, 4314, 4354, 4397, 4443, 4491, 4543, 4596,
     4652, 4711, 4771, 4833, 4897, 4962, 5029, 5097, 5165, 5235, 5305, 5375},
    {5446, 5564, 5681, 5795, 5907, 6014, 6116, 6212, 6301, 6383, 6457, 6524, 6583, 6633, 6676, 6711, 6740, 6761,
     6777, 6788, 6794, 6796, 6795, 6793, 6789, 6784, 6779, 6775, 6772, 6769, 6769, 6769, 6772, 6775, 6779, 6784,
     6789, 6793, 6795, 6796, 6794, 6788, 6777, 6761, 6740, 6711, 6676, 6633, 6583, 6524, 6457, 6383, 6301, 6212,
     6116, 6014, 5907, 5795, 5681, 5564, 5446, 5328, 5211, 5097, 4985, 4878, 4776, 4680, 4591, 4509, 4435, 4368,
     4309, 4259, 4216, 4181, 4152, 4131, 4115, 4104, 4098, 4096, 4097, 4099, 4103, 4108, 4113, 4117, 4120, 4123,
     4123, 4123, 4120, 4117, 4113, 4108, 4103, 4099, 4097, 4096, 4098, 4104, 4115, 4131, 4152, 4181, 4216, 4259,
     4309, 4368, 4435, 4509, 4591, 4680, 4776, 4878, 4985, 5097, 5211, 5328}
};

void MCP4922_2SineOutput(void) {
    DAC_A.Data11_0 = Sine[A][SineIndex];
    DAC_B.Data11_0 = Sine[B][SineIndex];
    SPINumberOfTx  = 2;
    nSS            = 0;
    SPI1BUF        = DAC_A.Int;
    if (++SineIndex >= 120) SineIndex = 0;
}
