/*
 * File:   main.c
 * Author: user
 *
 * Created on April 29, 2020, 2:47 PM
 */


#include <xc.h>
#include "Common.h"

#define pot_vaule ChannelVolt_mV[0]

/*timer1*/
extern uint16_t T1Cnt_1ms;
uint16_t TCnt_5ms = 0, TCnt_50ms = 0;

/*MultiTask*/
uint8_t TASK = 0;
uint8_t MultiTaskCnt_10ms = 0, MultiTaskCnt_100ms = 0, MultiTaskCnt_500ms = 0, MultiTaskCnt_1000ms = 0;

/*Seg*/
extern uint8_t FirstLineData[6];
extern uint8_t SecondLineData[6];
extern const uint8_t SevenSegPattern[16];

/*ADC*/
extern uint16_t VR1_8bit, ADC1BUF[8], ChannelVolt_mV[8];

/*USB CDC*/
uint8_t CDCSendBuffer[64];
uint8_t numbDatas;
uint8_t CDCRecvBuffer[64];
bool flagOpenUSBCDCSerial;

/*LED*/
uint8_t LED2Blink, LED2BlinkDuty, LED3BlinkDuty;

/*Buzzer*/
uint8_t BzTCnt;
uint8_t BzOutput;
uint8_t BzMode0Duty, BzMode0NumOfTime;
uint8_t BzMode1Duty, BzMode1NumOfTime;
uint8_t BzMode2Duty, BzMusicPointer;
uint8_t flagBzMode;

/*ECAN1*/
__eds__ ECAN1MessageBuffer_t ecan1msgBuf __attribute__((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));
uint32_t CANIdentifier;

/*Debounce*/
uint8_t SwitchOnCnt[10], SwitchOffCnt[10], SwitchDebounceTime = 20, PreviousRTCSecond, RTCSecond;

/*RTCC*/
RTCC_t Now;
uint8_t flagRTCCSetTime;
uint8_t flagRTCCSetTime1shot = 0;
uint8_t rtccSecondChanged = 0, rtccReadFailure = 0;

/*Beep*/
const uint8_t BzMusic[29] = {100, 100, 0, 0, 100, 100, 0, 0, 100, 0, 100, 0, 100, 100, 0, 0, 100, 0, 100, 0, 100, 0, 100, 100, 0, 0, 100, 0, 100};

/*SPI Data*/
extern const uint16_t Sine[2][120];
extern uint8_t SineIndex;

I2CLCDFlag_t I2CStatus;
I2CLCDIO_t I2CLCD;
I2CLCDAddress_t I2CAddress;
I2CDevice_t I2CDevice;

SwitchItem_t SwitchStatus, PreviousSwitchStatus;
TimeFlag_t FLAG;
BuzzerMode_t BzMode;
RealTimeClock_t CLOCK;
CANDataFrame_t CANDataFrame;

void System_Initialize(void);
void OSCILLATOR_Initialize(void);
void PinManager_Initialize(void);
void Time_Execute(void);
void CheckSwitch(void);
void Debounce(uint16_t Input, DebounceSwitch_t Switch);
void CheckBzMode(void);
void CheckLEDMode(void);
void LED2Stateflow(void);
void LED3Stateflow(void);
void UpdateClock(void);
void Time_Buf_to_LINE1(void);
void EXT_ADC_Buf_to_LINE2(void);

bool CheckI2CDevice(uint8_t ADDRESS, I2CModules_t MODULES);
void LCD_Initialize(void);
void LCD_WriteInstruction(uint8_t Instruction);
void LCD_WriteData(uint8_t DATA);
void LCD_SetCursor(uint8_t CurY, uint8_t CurX);
void LCD_PutROMString(const uint8_t *String);

void IIC_TEST(void);

/*
Main application
 */
int main(void) {
    // initialize the device
    System_Initialize();

    LCM_SetCursor(0, 2);
    LCM_PutROMString((const uint8_t*) "Hello, World");

    LCM_SetCursor(1, 0);
    LCM_PutROMString((const uint8_t*) "Son of San,Dick!");

    /*LCD_Initialize();
    LCD_SetCursor(0, 0);
    LCD_PutROMString((const uint8_t*) "Hello, World");*/

    I2CDevice.MCP79410 = CheckI2CDevice(SLAVE_I2C1_MCP79410_REG_ADDRESS, I2C1);
    I2CDevice.MCP4551 = CheckI2CDevice(SLAVE_I2C2_MCP4551_ADDRESS, I2C2);
    /* Disable Watch Dog Timer */
    RCONbits.SWDTEN = 1; //4ms
    if (I2CDevice.MCP4551 == 1 && I2CDevice.MCP79410 == 1) {
        BzMode._0 = 1;
    }
    while (1) {
        Time_Execute();
    }
}

void System_Initialize(void) {
#ifndef USING_SIMULATOR
    OSCILLATOR_Initialize();
    USBDeviceInit();
    USBDeviceAttach();
    flagOpenUSBCDCSerial = 1;
#endif
    PinManager_Initialize();
    Timer1_Initialize();
    UART1_Initialize();
    /*PWM_Initialize();*/
    ADC1_Initialize();
    ECAN1_Initialize();
    SPI1_Initialize();
    I2C1_Initialize();
    I2C2_Initialize();
    LCM_Initialize();

    LINE_12_Initialize();
}

void OSCILLATOR_Initialize(void) {
    /*  Configure Oscillator to operate the device at 40Mhz
     Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
     Fosc= 8M*40/(2*2)=80Mhz for 8M input clock */
    PLLFBD = 38; /* M=40 */
    CLKDIVbits.PLLPOST = 0; /* N1=2 */
    CLKDIVbits.PLLPRE = 0; /* N2=2 */
    OSCTUN = 0; /* Tune FRC oscillator, if FRC is used */

    // Configure APLL prescaler, APLL postscaler, APLL divisor
    ACLKCON3bits.ASRCSEL = 1; // Select Primary Oscillator as the clock source
    ACLKCON3bits.FRCSEL = 0; // Select Primary Oscillator as the clock source
    ACLKCON3bits.SELACLK = 1; // Select Primary PLL or oscillators to provide
    // the source clock for auxiliary clock divider
    ACLKDIV3bits.APLLDIV = 0b111; // M = 24
    ACLKCON3bits.APLLPRE = 0b001; // N1 = 2
    ACLKCON3bits.APLLPOST = 0b110; // N2 = 2
    ACLKCON3bits.ENAPLL = 1; // Enable Auxiliary Clock

    // Wait for APLL lock to occur
    while (ACLKCON3bits.APLLCK != 1);

    /* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(OSCCON || 0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b011);
    // Wait for Clock switch to occur
    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1);
}

void PinManager_Initialize(void) {
    LEDEnable(LED_D1);
    LEDEnable(LED_D2);
    LEDEnable(LED_D3);
    LEDEnable(LED_D4);

    LEDTurnOff(LED_D1);
    LEDTurnOff(LED_D2);
    LEDTurnOff(LED_D3);
    LEDTurnOff(LED_D4);

    /*Buzzer*/
    TRISGbits.TRISG13 = 0;
    Buzzer = 0; /*Rated Frequency 2400 ± 200Hz */

    /*BT*/
    TRISDbits.TRISD8 = 1; /*BT2*/
    TRISAbits.TRISA15 = 1; /*BT3*/
    TRISDbits.TRISD0 = 1; /*BT4*/
    TRISDbits.TRISD11 = 1; /*BT5*/

    /*DSW*/
    TRISAbits.TRISA9 = 1; /*DSW1*/
    TRISBbits.TRISB8 = 1; /*DSW2*/
    TRISBbits.TRISB9 = 1; /*DSW3*/
    TRISBbits.TRISB10 = 1; /*DSW4*/
    TRISBbits.TRISB11 = 1; /*DSW5*/
    TRISAbits.TRISA1 = 1; /*DSW6*/

    ANSELA &= 0x7DFD;
    ANSELB &= 0xF0FF;

    __builtin_write_OSCCONL(OSCCON & ~(1 << 6)); // unlock PPS

    /*UART1*/
    RPOR8bits.RP99R = 0b000001; //RF3->UART1:U1TX
    RPINR18bits.U1RXR = 0b1100010; //RF2->UART1:U1RX

    /*SPI1*/
    RPOR5bits.RP84R = 0x0006; //RE4->SPI1:SCK1
    RPOR5bits.RP82R = 0x0005; //RE2->SPI1:SDO1
    //RPOR4bits.RP79R     = 0x0007; //RD15->SPI1:SS1
    TRISDbits.TRISD15 = 0;
    RPINR20bits.SCK1R = 0x0054; //RE4->SPI1:SCK1

    TRISDbits.TRISD14 = 0; //RD14->\LDAC

    /*ECAN1*/
    RPINR26bits.C1RXR = 108; //set CAN1 RX to RP108  (40)
    RPOR12bits.RP109R = 14; //RPOR12bits.RP109R = 14; set CAN1TX to RP109(39)
    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // lock PPS

}

void Time_Execute(void) {
    if (FLAG.Second == 1) {
        UpdateClock();
        FLAG.Second = 0;
    }
    if (T1Cnt_1ms >= 20) {
        T1Cnt_1ms = 0x00;
        ClrWdt();
        CheckSwitch();
        Scan7Segment();
        MultiTask();
        CheckBzMode();
        CheckLEDMode();
        if (++TCnt_5ms >= 5) {
            TCnt_5ms = 0;
            Time_Buf_to_LINE1();
#ifndef USING_SIMULATOR
            MCP4922_2SineOutput();
            if (I2CDevice.MCP4551 == 1) {
                MCP4551_Command(Volatile_Wiper_0, WriteData, VR1_8bit);
            }
            if (flagOpenUSBCDCSerial == 1) {
                /*USB*/

                if (USBGetDeviceState() < CONFIGURED_STATE) {
                    return;
                }

                if (USBIsDeviceSuspended() == true) {
                    return;
                }

                if (USBUSARTIsTxTrfReady() == true) {
                    numbDatas = getsUSBUSART(CDCRecvBuffer, sizeof (CDCRecvBuffer)); //until the CDCRecvBuffer is free.
                    if (numbDatas == 3) {
                        Nop();
                        switch (CDCRecvBuffer[0]) {
                            case '0':
                                BzMode._0 = 1;
                                break;
                            case '1':
                                BzMode._1 = 1;
                                break;
                            case '2':
                                BzMode._2 = 1;
                                break;
                        }
                        LEDToggle(LED_D1);
                    }
                    uint8_t BufferPointer = 0;
                    CDCSendBuffer[BufferPointer++] = 0x44;
                    CDCSendBuffer[BufferPointer++] = pot_vaule & 0xff;
                    CDCSendBuffer[BufferPointer++] = (pot_vaule & 0xff00) >> 8;
                    CDCSendBuffer[BufferPointer++] = Sine[A][SineIndex]& 0xff;
                    CDCSendBuffer[BufferPointer++] = (Sine[A][SineIndex]& 0xff00) >> 8;
                    CDCSendBuffer[BufferPointer++] = Sine[B][SineIndex]& 0xff;
                    CDCSendBuffer[BufferPointer++] = (Sine[B][SineIndex]& 0xff00) >> 8;
                    CDCSendBuffer[BufferPointer++] = BzOutput;
                    CDCSendBuffer[BufferPointer++] = 0xff ^ CDCSendBuffer[0];
                    CDCSendBuffer[BufferPointer++] = '\r';
                    CDCSendBuffer[BufferPointer++] = '\n';
                    putUSBUSART(CDCSendBuffer, BufferPointer);
                }
                CDCTxService();
            }
#endif      
        }
    }
}

void MultiTask(void) {
    static uint8_t flagCAN_Tx1shot = 0, flagCAN_TxContinuous = 0, flagMusic1shot = 0, Count = 0;
    switch (TASK) {
        default:
            TASK = 0;
        case 0:
            if (SwitchStatus.bTACT != 0 || SwitchStatus.bDIP != 0) { /*Freq: 50Hz*/
                LEDTurnOn(LED_D4);
                if ((PreviousSwitchStatus.bTACT != SwitchStatus.bTACT) || (PreviousSwitchStatus.bDIP != SwitchStatus.bDIP)) flagCAN_Tx1shot = 1;
                if (MultiTaskCnt_1000ms <= 50 && SwitchStatus.bTACT != 0) ++MultiTaskCnt_1000ms;
                else if (SwitchStatus.bTACT == 0x00 && SwitchStatus.bDIP == 0x00) {
                    MultiTaskCnt_1000ms = 0;
                    flagCAN_TxContinuous = 0;
                } else if (MultiTaskCnt_1000ms > 50) flagCAN_TxContinuous = 1;
                //ID = 0x18ABCDEF
                if ((C1TR23CONbits.TXREQ3 == 0 && flagCAN_Tx1shot == 1) || flagCAN_TxContinuous) {
                    flagCAN_Tx1shot = 0;
                    CANDataFrame.byteData0 = CLOCK.Year;
                    CANDataFrame.byteData1 = CLOCK.Month;
                    CANDataFrame.byteData2 = CLOCK.Date;
                    CANDataFrame.byteData3 = CLOCK.Weekday;
                    CANDataFrame.byteData4 = CLOCK.Hour;
                    CANDataFrame.byteData5 = CLOCK.Minute;
                    CANDataFrame.byteData6 = CLOCK.Second;
                    CANDataFrame.byteData7 = 0x00;
                    Ecan1WriteTxMsgBufId(3, CANTX_ID_TRIGGER, 1, 0);
                    Ecan1WriteTxMsgBufData(3, 8,
                            (uint16_t) CANDataFrame.wordData0,
                            (uint16_t) CANDataFrame.wordData1,
                            (uint16_t) CANDataFrame.wordData2,
                            (uint16_t) CANDataFrame.wordData3);
                    C1TR23CONbits.TXREQ3 = 1;
                }
            } else {
                LEDTurnOff(LED_D4);
                flagCAN_Tx1shot = 0;
                flagCAN_TxContinuous = 0;
                MultiTaskCnt_1000ms = 0;
            }
            PreviousSwitchStatus.bTACT = SwitchStatus.bTACT;
            PreviousSwitchStatus.bDIP = SwitchStatus.bDIP;
            Nop();
            break;
        case 1:
            if (SwitchStatus.TACT._BT3 == 1) {
                if (flagMusic1shot == 0) BzMode._2 = 1;
                flagMusic1shot = 1;
            } else flagMusic1shot = 0;
            if (pot_vaule >= 3150 ||
                    SwitchStatus.TACT._BT5 != 0 ||
                    (CANIdentifier == (CANRX_ID_1 + 1) && SwitchStatus.DIP._DSW6 == 1) ||
                    flagBzMode) BzOutput = 1;
            else BzOutput = 0;
            Nop();
            break;
        case 2:
            Nop();
            break;
        case 3:
            Nop();
            break;
        case 4:
            Nop();
            break;
        case 5:
            if (flagRTCCSetTime == 1 &&
                    flagRTCCSetTime1shot == 0 &&
                    I2CDevice.MCP79410 == 1) {
                MCP79410_Initialize();
                BzMode._1 = 1;
                MCP79410_DisableOscillator();
                MCP79410_SetTime(&Now);
                MCP79410_EnableOscillator(); //Start clock by enabling oscillator
                MCP79410_GetTime();
                flagRTCCSetTime1shot = 1;
            }
            Nop();
            break;
        case 6:
            if (++MultiTaskCnt_100ms >= 5) { /*Freq: 10Hz*/
                MultiTaskCnt_100ms = 0;
                //ID = 0x405
                if (C1TR01CONbits.TXREQ1 == 0) {
                    Ecan1WriteTxMsgBufId(1, CANTX_ID_100MS, 1, 0);
                    Ecan1WriteTxMsgBufData(1, 2,
                            (uint16_t) pot_vaule,
                            (uint16_t) 0x00,
                            (uint16_t) 0x00,
                            (uint16_t) 0x00);
                    C1TR01CONbits.TXREQ1 = 1;
                }
            }
            Nop();
            break;
        case 7:
            Nop();
            break;
        case 8:
            Nop();
            break;
        case 9:
            Nop();
            break;
        case 10:
            if (++Count >= 5) {
                Count = 0;
                IIC_TEST();
            }
            Nop();
            break;
        case 11:
            if (++MultiTaskCnt_500ms >= 25) { /*Freq: 2Hz*/
                MultiTaskCnt_500ms = 0;
                //ID = 0x200
                if (C1TR23CONbits.TXREQ2 == 0) {
                    Ecan1WriteTxMsgBufId(2, CANTX_ID_500MS, 1, 0);
                    Ecan1WriteTxMsgBufData(2, 8,
                            (uint16_t) 0x2301,
                            (uint16_t) 0x4523,
                            (uint16_t) 0x6745,
                            (uint16_t) 0x0167);
                    C1TR23CONbits.TXREQ2 = 1;
                }
            }
            Nop();
            break;
        case 12:
            Nop();
            break;
        case 13:
            Nop();
            break;
        case 14:
            Nop();
            break;
        case 15:
            Nop();
            break;
        case 16:
            Nop();
            break;
        case 17:
            Nop();
            break;
        case 18:
            Nop();
            break;
        case 19:
            Nop();
            break;
    }
    if (TASK % 10 == 2) {
        EXT_ADC_Buf_to_LINE2(); /*Freq: 100Hz*/
    } else if (TASK % 10 == 3) {
        static uint8_t rtccSecondChanged1shot = 0;
#ifndef USING_SIMULATOR
        if (I2CDevice.MCP79410 == 1 && rtccSecondChanged == 0 && rtccReadFailure == 0) {
            uint8_t I2CRetryCnt = 0;
            static uint8_t _1shot = 0;
            if (_1shot == 0) {
                _1shot = 1;
                RTCSecond = MCP79410_Command(RTCSEC, 0x00, Read);
                PreviousRTCSecond = RTCSecond;
            }
            RTCSecond = MCP79410_Command(RTCSEC, 0x00, Read);
            if (PreviousRTCSecond != RTCSecond) {
                rtccSecondChanged = 1;
            }
            if (I2CDevice.MCP79410 != 1 || I2CRetryCnt == 100) {
                rtccReadFailure = 1;
            } else {
                I2CRetryCnt++;
            }
            PreviousRTCSecond = RTCSecond;
        } else if (I2CDevice.MCP79410 == 0) {
            rtccReadFailure = 1;
        }
        if (rtccSecondChanged == 1 && rtccSecondChanged1shot == 0) {
            MCP79410_GetTime();
            rtccSecondChanged1shot = 1;
        }
#else
        rtccReadFailure = 1;
#endif
    }
    if (TASK % 5 == 0) {
        if (++MultiTaskCnt_10ms >= 2) {
            MultiTaskCnt_10ms = 0;
            //ID = 0x400
            if (C1TR01CONbits.TXREQ0 == 0) {
                Ecan1WriteTxMsgBufId(0, CANTX_ID_20MS, 1, 0);
                Ecan1WriteTxMsgBufData(0, 1,
                        (uint16_t) CLOCK.Second,
                        (uint16_t) 0x00,
                        (uint16_t) 0x00,
                        (uint16_t) 0x00);
                C1TR01CONbits.TXREQ0 = 1;
            }
        }
    }
    TASK++;
}

void UpdateClock(void) {
    CLOCK.Second++;
    if (CLOCK.Second >= 60) {
        CLOCK.Second = 0;
        CLOCK.Minute++;
        if (I2CDevice.MCP79410 == 1) {
            MCP79410_GetTime();
        }
        if (SwitchStatus.bDIP == 0x00) {
            if ((CLOCK.Hour == 17 && CLOCK.Minute == 0) || (CLOCK.Hour == 11 && CLOCK.Minute == 25)) BzMode._2 = 1;
            else if (CLOCK.Hour == 11 && CLOCK.Minute == 0) BzMode._1 = 1;
        }
        if (CLOCK.Minute >= 60) {
            CLOCK.Minute = 0;
            CLOCK.Hour++;
            if (CLOCK.Hour >= 24)
                CLOCK.Hour = 0;
        }
    }
}

void Time_Buf_to_LINE1(void) {
    if (rtccSecondChanged == 1 || rtccReadFailure == 1) {
        FirstLineData[0] = SevenSegPattern[CLOCK.Hour / 10];
        FirstLineData[1] = SevenSegPattern[CLOCK.Hour % 10];
        FirstLineData[2] = SevenSegPattern[CLOCK.Minute / 10];
        FirstLineData[3] = SevenSegPattern[CLOCK.Minute % 10];
        FirstLineData[4] = SevenSegPattern[CLOCK.Second / 10];
        FirstLineData[5] = SevenSegPattern[CLOCK.Second % 10];
    } else {
        FirstLineData[0] = SevenSegPattern[10];
        FirstLineData[1] = SevenSegPattern[10];
        FirstLineData[2] = SevenSegPattern[10];
        FirstLineData[3] = SevenSegPattern[10];
        FirstLineData[4] = SevenSegPattern[10];
        FirstLineData[5] = SevenSegPattern[10];
    }

}

void EXT_ADC_Buf_to_LINE2(void) {
    uint16_t Temp_Value;

    Temp_Value = pot_vaule;
    SecondLineData[0] = SevenSegPattern[Temp_Value / 1000 ];
    Temp_Value = Temp_Value - (1000 * (Temp_Value / 1000));
    SecondLineData[1] = SevenSegPattern[Temp_Value / 100];
    Temp_Value = Temp_Value - (100 * (Temp_Value / 100));
    SecondLineData[2] = SevenSegPattern[Temp_Value / 10];
    SecondLineData[3] = SevenSegPattern[Temp_Value % 10];
    SecondLineData[4] = SevenSegPattern[10];
    SecondLineData[5] = SevenSegPattern[10];
}

void CheckSwitch(void) {
    Debounce(BT2, bt2);
    Debounce(BT3, bt3);
    Debounce(BT4, bt4);
    Debounce(BT5, bt5);

    Debounce(DSW1, dsw1);
    Debounce(DSW2, dsw2);
    Debounce(DSW3, dsw3);
    Debounce(DSW4, dsw4);
    Debounce(DSW5, dsw5);
    Debounce(DSW6, dsw6);

}

void Debounce(uint16_t Input, DebounceSwitch_t Switch) {
    uint16_t* Output;
    uint8_t BitOn, BitOff;
    if (Switch >= bt2 && Switch <= bt5) {
        BitOn = Switch;
        Output = (uint16_t *) & SwitchStatus.bTACT;
    } else if (Switch >= dsw1 && Switch <= dsw6) {
        BitOn = (Switch - dsw1);
        Output = (uint16_t *) & SwitchStatus.bDIP;
    }
    BitOff = ~(1 << BitOn);
    BitOn = (1 << BitOn);
    if (Input) {
        SwitchOffCnt[Switch] = 0x00;
        if (SwitchOnCnt[Switch] < SwitchDebounceTime) ++SwitchOnCnt[Switch];
        else *Output |= BitOn;
    } else {
        SwitchOnCnt[Switch] = 0x00;
        if (SwitchOffCnt[Switch] < SwitchDebounceTime) ++SwitchOffCnt[Switch];
        else *Output &= BitOff;
    }
}

void CheckLEDMode(void) {
    LED2Stateflow();
    LED3Stateflow();
}

void LED2Stateflow(void) {
    static uint8_t State = 0;
    static uint16_t count = 0;
    if (LED2Blink == 1) {
        switch (State) {
            default:
                LED2BlinkDuty = 100;
                State = 0;
            case 0:
                if (count < 50) count++;
                else {
                    count = 0;
                    LED2BlinkDuty = 5;
                    State++;
                }
                break;
            case 1:
                if (count < 50) count++;
                else {
                    count = 0;
                    LED2BlinkDuty = 100;
                    State++;
                }
                break;
            case 2:
                if (count < 50) count++;
                else {
                    count = 0;
                    LED2Blink = 0;
                    State++;
                }
                break;
        }
    } else {
        LED2BlinkDuty = 5;
    }
}

void LED3Stateflow(void) {
    static uint8_t State = 0;
    static uint16_t count = 0;
    switch (State) {
        default:
            State = 0;
        case 0:
            if (count < 10) count++;
            else {
                count = 0;
                if (++LED3BlinkDuty >= 100) State++;
            }
            break;
        case 1:
            if (count < 10) count++;
            else {
                count = 0;
                if (--LED3BlinkDuty <= 0) State++;
            }
            break;
        case 2:
            if (count < 2000) count++;
            else {
                count = 0;
                State++;
            }
            break;
    }
}

void CheckBzMode(void) {
    if (BzMode._0) {
        if (++BzMode0Duty >= 100) {
            ++BzMode0NumOfTime;
            BzMode0Duty = 0;
        }
    } else {
        BzMode0Duty = 0;
        BzMode0NumOfTime = 0;
    }
    if (BzMode0NumOfTime >= 5) BzMode._0 = 0;
    /**************************************/

    if (BzMode._1) {
        if (++BzMode1Duty >= 200) {
            ++BzMode1NumOfTime;
            BzMode1Duty = 0;
        }
    } else {
        BzMode1Duty = 0;
        BzMode1NumOfTime = 0;
    }
    if (BzMode1NumOfTime >= 2) BzMode._1 = 0;
    if (BzMode._2) {
        if (++BzMode2Duty >= 100) {
            ++BzMusicPointer;
            BzMode2Duty = 0;
        }
    } else {
        BzMode2Duty = 0;
        BzMusicPointer = 0;
    }
    if (BzMusicPointer >= 29) BzMode._2 = 0;
    /**************************************/

    if ((BzMode0Duty < 50 && BzMode._0) ||
            (BzMode1Duty < 150 && BzMode._1) ||
            (BzMode2Duty < BzMusic[BzMusicPointer] && BzMode._2)) {
        flagBzMode = 1;
    } else {
        flagBzMode = 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS2bits.C1IF = 0; // clear interrupt flag
    if (C1INTFbits.TBIF) {
        C1INTFbits.TBIF = 0;
    }

    if (C1INTFbits.RBIF) {
        C1INTFbits.RBIF = 0;
        if (C1RXFUL1bits.RXFUL4) { //ID = 0x40
            mData(CANDataFrame, 4);
            C1RXFUL1bits.RXFUL4 = 0;
        }
        if (C1RXFUL1bits.RXFUL6) { //ID = 0x18ABCDEF
            mData(CANDataFrame, 6);
            Now.year = CANDataFrame.byteData0;
            Now.month = CANDataFrame.byteData1;
            Now.date = CANDataFrame.byteData2;
            Now.weekday = CANDataFrame.byteData3;
            Now.hour = CANDataFrame.byteData4;
            Now.min = CANDataFrame.byteData5;
            Now.sec = CANDataFrame.byteData6;
            if (CANDataFrame.byteData7 != 0) flagRTCCSetTime = 1;
            C1RXFUL1bits.RXFUL6 = 0;
        }
        if (C1RXFUL1bits.RXFUL7) { //ID = 0x30
            mData(CANDataFrame, 7);
            if (CANDataFrame.EID & 0x01) LEDToggle(LED_D3);
            C1RXFUL1bits.RXFUL7 = 0;
        }
        CANIdentifier = ((uint32_t) CANDataFrame.SID << 18) + CANDataFrame.EID;
        Nop();
    }
}

bool CheckI2CDevice(uint8_t ADDRESS, I2CModules_t MODULES) {
    uint16_t timeout = 0;
    I2C1_MESSAGE_STATUS I2C1MessageStatus;
    I2C2_MESSAGE_STATUS I2C2MessageStatus;
    if (MODULES == I2C1) {
        I2C1_MasterWrite(0x00, 0x00, ADDRESS, &I2C1MessageStatus);
        while (I2C1MessageStatus == I2C1_MESSAGE_PENDING) {
            if (timeout == I2C_MAX_TRY) {
                break;
            } else {
                ++timeout;
            }
        }
        if (I2C1MessageStatus == I2C1_MESSAGE_COMPLETE) {
            return true;
        }
    } else if (MODULES == I2C2) {
        I2C2_MasterWrite(0x00, 0x00, ADDRESS, &I2C2MessageStatus);
        while (I2C2MessageStatus == I2C2_MESSAGE_PENDING) {
            if (timeout == I2C_MAX_TRY) {
                break;
            } else {
                ++timeout;
            }
        }
        if (I2C2MessageStatus == I2C2_MESSAGE_COMPLETE) {
            return true;
        }
    }
    return false;
}

void LCD_Initialize(void) {
    I2CLCD.BL = I2CStatus.Enale = 1;
    if (I2CStatus.Enale) {
        I2CStatus.n8bit_4bit = 1;
        LCD_WriteInstruction(0x3);
#ifndef USING_SIMULATOR
        __delay_us(3000); /*wait time > 4.5ms*/
#endif

        LCD_WriteInstruction(0x3);
#ifndef USING_SIMULATOR
        __delay_us(1500); /*wait time > 1.5ms*/
#endif

        LCD_WriteInstruction(0x3);
#ifndef USING_SIMULATOR
        __delay_us(100); /*wait time > 100us*/
#endif

        LCD_WriteInstruction(0x2);
#ifndef USING_SIMULATOR
        __delay_us(100); /*wait time > 100us*/
#endif

        I2CStatus.n8bit_4bit = 0;
    }
    LCD_WriteInstruction(LCD_FOUR_BIT);
#ifndef USING_SIMULATOR
    __delay_us(100); /*wait time > 100us*/
#endif

    LCD_WriteInstruction(LCD_CURSOR_ON);
#ifndef USING_SIMULATOR
    __delay_us(1800); /*wait time > 1.6ms*/
#endif

}

void LCD_WriteInstruction(uint8_t Instruction) {
    uint8_t LCD_Instruction[4];
    uint8_t *Temporary = LCD_Instruction;
    uint8_t length = 2;
    uint16_t i2c_PendingTimeout;
    //I2C1_MESSAGE_STATUS I2C1MessageStatus;
    I2C2_MESSAGE_STATUS I2C2MessageStatus;
    I2CLCD.RS = 0;
    I2CLCD.RW = 0;
    I2CAddress.Address = SLAVE_I2C_LCD_ADDRESS;
    I2CAddress.R_nW = 0;
    if (!I2CStatus.n8bit_4bit) {
        I2CLCD.D7_D4 = ((Instruction & 0xF0) >> 4) &0x0F;
        I2CLCD.EN = 1;
        *Temporary++ = I2CLCD.byteData;
        I2CLCD.EN = 0;
        *Temporary++ = I2CLCD.byteData;
        length = 4;
    }
    I2CLCD.D7_D4 = (Instruction & 0x0F);
    I2CLCD.EN = 1;
    *Temporary++ = I2CLCD.byteData;
    I2CLCD.EN = 0;
    *Temporary++ = I2CLCD.byteData;
    I2C2_MasterWrite(LCD_Instruction, length, I2CAddress.Address, &I2C2MessageStatus);
    i2c_PendingTimeout = 0;
#ifndef USING_SIMULATOR
    while (I2C2MessageStatus == I2C2_MESSAGE_PENDING) {
        if (i2c_PendingTimeout == 1500) {
            break;
        } else i2c_PendingTimeout++;
    }
#endif
}

void LCD_WriteData(uint8_t DATA) {
    uint8_t LCD_Data[4];
    uint8_t *Temporary = LCD_Data;
    uint8_t length = 4;
    uint16_t i2c_PendingTimeout;
    //I2C1_MESSAGE_STATUS I2C1MessageStatus;
    I2C2_MESSAGE_STATUS I2C2MessageStatus;
    I2CLCD.RS = 1;
    I2CLCD.RW = 0;
    I2CAddress.Address = SLAVE_I2C_LCD_ADDRESS;
    I2CAddress.R_nW = 0;
    I2CLCD.D7_D4 = ((DATA & 0xF0) >> 4) &0x0F;
    I2CLCD.EN = 1;
    *Temporary++ = I2CLCD.byteData;
    I2CLCD.EN = 0;
    *Temporary++ = I2CLCD.byteData;

    I2CLCD.D7_D4 = (DATA & 0x0F);
    I2CLCD.EN = 1;
    *Temporary++ = I2CLCD.byteData;
    I2CLCD.EN = 0;
    *Temporary++ = I2CLCD.byteData;
    I2C2_MasterWrite(LCD_Data, length, I2CAddress.Address, &I2C2MessageStatus);
    i2c_PendingTimeout = 0;
#ifndef USING_SIMULATOR
    while (I2C2MessageStatus == I2C2_MESSAGE_PENDING) {
        if (i2c_PendingTimeout == 1500) {
            break;
        } else i2c_PendingTimeout++;
    }
#endif
}

void LCD_PutROMString(const uint8_t *String) {
    while (*String != 0x00) {
        LCD_WriteData(*String++);
#ifndef USING_SIMULATOR
        __delay_us(50);
#endif
    }
}

void LCD_SetCursor(uint8_t CurY, uint8_t CurX) {
    LCD_WriteInstruction(0x80 + CurY * 0x40 + CurX);
#ifndef USING_SIMULATOR
    __delay_us(50);
#endif
}

void IIC_TEST(void) {
    I2C2_MESSAGE_STATUS I2C2MessageStatus;
    uint8_t bLED[2] = {0x87, 0x00};
    uint16_t i2c_PendingTimeout;
    static uint8_t RollingCount = 0;
    static bool nRising_Falling = 0;
    if (nRising_Falling == 1) {
        bLED[1] = --RollingCount;
    } else {
        bLED[1] = ++RollingCount;
    }
    if (RollingCount <= 0 || RollingCount >= 255) {
        nRising_Falling ^= 1;
    }
    I2C2_MasterWrite(bLED, 2, 0x4F, &I2C2MessageStatus);
    i2c_PendingTimeout = 0;
#ifndef USING_SIMULATOR
    while (I2C2MessageStatus == I2C2_MESSAGE_PENDING) {
        if (i2c_PendingTimeout == 1500) {
            break;
        } else i2c_PendingTimeout++;
    }
#endif
}
/**
 End of File
 */
