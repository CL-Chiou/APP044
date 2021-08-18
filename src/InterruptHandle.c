#include "Common.h"

void ADC1_DMA2DefInterruptHandler(void) {
#include "adc1.h"
    extern uint16_t ADC1CH0123_mV[4];
    extern __eds__ uint16_t ADC1BUF[8] __attribute__((eds, space(dma)));
    extern uint16_t VR1_8bit;
    ADC1CH0123_mV[3] = mADCToVoltage_3300mV(ADC1BUF[AN3]); // Read the AN3:ANIN2 conversion result
    ADC1CH0123_mV[0] = mADCToVoltage_3300mV(ADC1BUF[AN0]); // Read the AN0:VR1_8bit conversion result
    ADC1CH0123_mV[1] = mADCToVoltage_3300mV(ADC1BUF[AN1]); // Read the AN1:ANIN4 conversion result
    ADC1CH0123_mV[2] = mADCToVoltage_3300mV(ADC1BUF[AN2]); // Read the AN2:ANIN3 conversion result
    //    DMA0CONbits.CHEN = 1; // Enable DMA0 channel
    //    DMA0REQbits.FORCE = 1; // Manual mode:Kick-start the 1st transfer
    VR1_8bit = ADC1BUF[AN0] >> 2;
    MDC = ADC1CH0123_mV[0];
}

void ADC1_AD1DefInterruptHandler(void) {
#include "adc1.h"
    extern uint16_t ADC1CH0123_mV[4], U1Tx_PotVolt_mV[U1TX_MSG_BUF_LENGTH];
    ADC1CH0123_mV[0] = mADCToVoltage_3300mV(ADC1BUF0); // Read the VR1_8bit conversion result
    ADC1CH0123_mV[1] = mADCToVoltage_3300mV(ADC1BUF1); // Read the ANIN4 conversion result
    ADC1CH0123_mV[2] = mADCToVoltage_3300mV(ADC1BUF2); // Read the ANIN3 conversion result
    ADC1CH0123_mV[3] = mADCToVoltage_3300mV(ADC1BUF3); // Read the ANIN2 conversion result
    ADC1CH0123_mV[4] = mADCToVoltage_3300mV(ADC1BUF4); // Read the ANIN1 conversion result
    U1Tx_PotVolt_mV[0] = (ADC1CH0123_mV[0] / 1000) + '0';
    U1Tx_PotVolt_mV[1] = (ADC1CH0123_mV[0] / 100 % 10) + '0';
    U1Tx_PotVolt_mV[2] = (ADC1CH0123_mV[0] / 10 % 10) + '0';
    U1Tx_PotVolt_mV[3] = (ADC1CH0123_mV[0] % 10) + '0';
#ifdef USING_SIMULATOR
    extern uint8_t *pU1String, U1TxSimMsg[U1TX_MSG_BUF_LENGTH];
    pU1String = U1TxSimMsg;
    IFS0bits.U1TXIF = 1;
#else
    DMA0CONbits.CHEN = 1; // Enable DMA0 channel
    DMA0REQbits.FORCE = 1; // Manual mode:Kick-start the 1st transfer
#endif
}

void SPI1_DefInterruptHandler(void) {
#include "spi1.h"
#include "MCP4922.h"
    extern uint8_t SPINumberOfTx;
    extern mcp4922cmd DAC_B;
    nSS = 1;
    Nop();
    Nop();
    nLDAC = 0;
    Nop();
    Nop();
    Nop();
    nLDAC = 1;
    if (--SPINumberOfTx) {
        Nop();
        Nop();
        Nop();
        nSS = 0;
        SPI1BUF = DAC_B.Int;
    }
}

void Timer1_DefInterruptHandler(void) {
#include "Timers.h"
#include "MP_Car_leds.h"
    extern uint16_t T1Cnt, T1Cnt_1ms, T1Cnt_1000ms;
    extern uint8_t T1Cnt_RLED_Period, T1Cnt_OLED_Period;
    extern uint8_t BzTCnt, BzOutput;
    extern uint8_t LED2Blink, LED2BlinkDuty, LED3BlinkDuty;
    extern uint8_t rtccSecondChanged, rtccReadFailed;
    extern xTimeFlag_t TimeFlag;
    T1Cnt++;
    T1Cnt_1ms++;
    T1Cnt_RLED_Period++;
    T1Cnt_OLED_Period++;
    BzTCnt++;
    if (T1Cnt >= 10000) {
        T1Cnt = 0;
        T1Cnt_1000ms++;
        if (T1Cnt_1000ms >= 2) {
            T1Cnt_1000ms = 0;
            LED2Blink = 1;
        }
    }
    if (rtccSecondChanged == 1 || rtccReadFailed == 1) {
        static uint16_t OneSecondCnt = 0;
        if (OneSecondCnt >= 20000) {
            OneSecondCnt = 0;
            TimeFlag.Second = 1;
        } else {
            OneSecondCnt++;
        }
    }
    if (T1Cnt_RLED_Period >= 100) T1Cnt_RLED_Period = 0;
    if (T1Cnt_RLED_Period < LED2BlinkDuty) {
        LEDTurnOn(LED_D2);
    } else {
        LEDTurnOff(LED_D2);
    }

    if (T1Cnt_OLED_Period >= 100) T1Cnt_OLED_Period = 0;
    if (T1Cnt_OLED_Period < LED3BlinkDuty) {
        LEDTurnOn(LED_D3);
    } else {
        LEDTurnOff(LED_D3);
    }
    /* Rated Frequency: 2400 ± 200, Duty: 50% */
    if (BzTCnt <= 4 && BzOutput && configUSE_BUZZER) Buzzer = 1;
    else Buzzer = 0;
    if (BzTCnt >= 8) BzTCnt = 0;
}

void Timer3_DefInterruptHandler(void) {
    extern uint16_t TCnt_50ms;
    if (++TCnt_50ms >= 50) {
        TCnt_50ms = 0x00;
        DMA0CONbits.CHEN = 1; // Enable DMA0 channel
        DMA0REQbits.FORCE = 1; // Manual mode:Kick-start the 1st transfer
    }
}

void UART1_U1DefInterruptHandler(void) {
    extern uint8_t *pU1String;
    if (*pU1String != 0) {
        U1TXREG = *pU1String++; // Transmit one character  
    } else {
        Nop();
    }
}

void UART1_DMA0DefInterruptHandler(void) {
    extern uint16_t U1Tx_Protocol[16], U1Tx_PotVolt_mV[U1TX_MSG_BUF_LENGTH], ADC1CH0123_mV[4];
    extern xSwitchItem_t SwitchStatus;
    static uint16_t BufferCount = 0; // Keep record of which buffer// contains TX Data
    if (BufferCount == 0) {
        U1Tx_Protocol[0] = 0X0D;
        U1Tx_Protocol[1] = ADC1CH0123_mV[0] >> 8;
        U1Tx_Protocol[2] = ADC1CH0123_mV[0]&0xFF;
        //U1Tx_SanProtocol[3] = U1Tx_SanProtocol[0]^U1Tx_SanProtocol[1]^U1Tx_SanProtocol[2];
        U1Tx_Protocol[3] = ((U1Tx_Protocol[0]^U1Tx_Protocol[1]^U1Tx_Protocol[2]) + (SwitchStatus.byteDIP != 0 ? 1 : 0)) & 0xFF;
        U1Tx_Protocol[4] = 0x0C;
        DMA0STAL = __builtin_dmaoffset(U1Tx_Protocol);
        DMA0STAH = 0x0000;
        DMA0CNT = 4;
    } else {
        U1Tx_PotVolt_mV[0] = (ADC1CH0123_mV[0] / 1000) + '0';
        U1Tx_PotVolt_mV[1] = (ADC1CH0123_mV[0] / 100 % 10) + '0';
        U1Tx_PotVolt_mV[2] = (ADC1CH0123_mV[0] / 10 % 10) + '0';
        U1Tx_PotVolt_mV[3] = (ADC1CH0123_mV[0] % 10) + '0';
        DMA0STAL = __builtin_dmaoffset(U1Tx_PotVolt_mV);
        DMA0STAH = 0x0000;
        DMA0CNT = 7;
    }
    BufferCount ^= 1;
}
