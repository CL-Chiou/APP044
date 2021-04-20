/*
 * File:   main.c
 * Author: user
 *
 * Created on April 29, 2020, 2:47 PM
 */


#include <xc.h>
#include "Common.h"

/*i2c*/
extern I2C1_MESSAGE_STATUS i2c1_msg_status;
I2C2_MESSAGE_STATUS i2c2_msg_status;
/*timer1*/
extern uint16_t T1Counter_1ms;
uint16_t _10ms = 0, _100ms = 0, _500ms = 0;
/*Seg*/
extern uint8_t FIRST_LINE_Data[6];
extern uint8_t SECOND_LINE_Data[6];
extern const uint8_t SEVEN_SEG_PATTERN[16];
/*MCP4551 receive data*/
uint8_t RcvData[2];
/*ADC*/
extern uint16_t VR1, ADCValues_BUFFER[8];
extern int ADCValues[8];
/*USB CDC*/
char CDC_Buffer[64];
uint8_t Device_USB;
/*Buzzer*/
uint8_t BuzzerTimeCNT;
uint8_t BuzzerFlag;
uint8_t BzAlarm0, BzTrigger0;
uint8_t BzAlarm1, BzTrigger1;
uint8_t BzAlarm2, BzTrigger2;
uint8_t bz;
/*ECAN1*/
__eds__ ECAN1MSGBUF ecan1msgBuf __attribute__((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));
uint32_t CANidentifier;
/*Debounce*/
uint8_t Switch_DeBounce_On[10], Switch_DeBounce_Off[10], Deboune = 20, TimeChanged = 99;
/*RTCC*/
extern RTCC_Struct CurrentTime;
uint8_t SetTime;
uint8_t SetTime1Shot = 0;
/*Beep*/
const uint8_t Music_Beep[29] = {100, 100, 0, 0, 100, 100, 0, 0, 100, 0, 100, 0, 100, 100, 0, 0, 100, 0, 100, 0, 100, 0, 100, 100, 0, 0, 100, 0, 100};

/*BT*/
#define BT2 !PORTDbits.RD8 /*BT2*/
#define BT3 !PORTAbits.RA15 /*BT3*/
#define BT4 !PORTDbits.RD0 /*BT4*/
#define BT5 !PORTDbits.RD11 /*BT5*/

/*SW-DIP*/
#define DSW1 !PORTAbits.RA9 /*DSW1*/
#define DSW2 !PORTBbits.RB8 /*DSW2*/
#define DSW3 !PORTBbits.RB9 /*DSW3*/
#define DSW4 !PORTBbits.RB10 /*DSW4*/
#define DSW5 !PORTBbits.RB11 /*DSW5*/
#define DSW6 !PORTAbits.RA1 /*DSW6*/

/* Display ON/OFF Control defines */
#define DON                     0b00001111  /* Display on      */
#define DOFF                    0b00001011  /* Display off     */
#define CURSOR_ON               0b00001111  /* Cursor on       */
#define CURSOR_OFF              0b00001101  /* Cursor off      */
#define BLINK_ON                0b00001111  /* Cursor Blink    */
#define BLINK_OFF               0b00001110  /* Cursor No Blink */
#define BLINK_OFF_CURSOR_OFF    0b00001100  /* Cursor No Blink, Cursor off */
#define RETURN_HOME             0b00000001  /* Sets DDRAM address 0 in address counter */

/* Cursor or Display Shift defines */
#define SHIFT_CUR_LEFT          0b00000100  /* Cursor shifts to the left   */
#define SHIFT_CUR_RIGHT         0b00000101  /* Cursor shifts to the right  */
#define SHIFT_DISP_LEFT         0b00000110  /* Display shifts to the left  */
#define SHIFT_DISP_RIGHT        0b00000111  /* Display shifts to the right */

/* Function Set defines */
#define FOUR_BIT                0b00101100  /* 4-bit Interface               */
#define EIGHT_BIT               0b00111100  /* 8-bit Interface               */
#define LINE_5X7                0b00110000  /* 5x7 characters, single line   */
#define LINE_5X10               0b00110100  /* 5x10 characters               */
#define LINES_5X7               0b00111000  /* 5x7 characters, multiple line */

uint8_t I2C_Check_Device(uint8_t ADDRESS, i2c_modules MODULES);
void initial_LCD(void);
void LCD_WriteInstruction(uint8_t Instruction);
void LCD_WriteData(uint8_t DATA);
void LCD_Set_Cursor(uint8_t CurY, uint8_t CurX);
void LCD_PutROMString(const uint8_t *String);

struct _i2c_status I2C_STATUS;
union _i2c_lcd I2C_LCD;
union _i2c_address I2C_ADRRESS;
union _i2c_device I2C_Device;

struct _switch SW, _SW;
struct _tdm TDM;
union _mission Mission;
struct _flag FLAG;
struct _buzzeralarm BuzzerAlarm;
struct _clock CLOCK;
struct _frame_buffer FRAME_Buffer;

// *****************************************************************************
// Section: Static Function declaration
// *****************************************************************************

void OscConfig(void);
void IO_Config(void);
void Time_Execute(void);
void Mission_Execute(void);
void Check_Button(void);
void Debounce_Execute(uint16_t Input, DebounceSW Switch);
void Buzzer_Execute(void);
void Update_CLOCK1(void);
void Time_Buf_to_LINE1(void);
void EXT_ADC_Buf_to_LINE2(void);

int main(void) {
#ifdef USING_SIMULATOR
    __C30_UART = 1;
    fprintf(stdout, "USING_SIMULATOR: reseted\n");
#else
    /*SYSTEM_Initialize(SYSTEM_STATE_USB_START);
    USBDeviceInit();
    USBDeviceAttach();
    CDCSetLineCoding(9600, NUM_STOP_BITS_1, PARITY_NONE, 8);
    Device_USB = 1;*/
#endif
    OscConfig();
    IO_Config();
    Timer1_Initial();
    UART1_Initial();
    /*PWM_Initial();*/
    ADC1_Initial();
    Ecan1Init();
    SPI1_Initialize();
    I2C1_Initialize();
    I2C2_Initialize();
    LCM_Init();

    LINE_12_Initial();
    LINE_12_Write_Default();

    LCM_SetCursor(0, 2);
    LCM_PutROMString((const uint8_t*) "Hello, World");

    LCM_SetCursor(1, 0);
    LCM_PutROMString((const uint8_t*) "Son of San,Dick!");

    /*initial_LCD();
    LCD_Set_Cursor(0, 0);
    LCD_PutROMString((const uint8_t*) "Hello, World");*/

    //BuzzerAlarm._0 = 1;
    I2C_Device.MCP79410 = I2C_Check_Device(SLAVE_I2C1_MCP79410_REG_ADDRESS, I2C1);
    I2C_Device.MCP4551 = I2C_Check_Device(SLAVE_I2C2_MCP4551_ADDRESS, I2C2);
#ifndef USING_SIMULATOR
    if (I2C_Device.MCP79410 == 1) {
        static uint16_t RETRY = 100, retry_counter = 0;
        CurrentTime.sec = MCP79410_bcd2dec(MCP79410_Command(RTCSEC, 0x00, Read) & (~START_32KHZ));
        do {
            retry_counter++;
            TimeChanged = CurrentTime.sec;
            CurrentTime.sec = MCP79410_bcd2dec(MCP79410_Command(RTCSEC, 0x00, Read) & (~START_32KHZ));
            if (TimeChanged != CurrentTime.sec || I2C_Device.MCP79410 != 1 || retry_counter == RETRY) break;
            else __delay_ms(10);
        } while (CurrentTime.sec == TimeChanged);
    }
    __delay_ms(10);
    MCP79410_GetTime();
#endif
    while (1) {
        Time_Execute();
    }
}

void OscConfig(void) {
    /*  Configure Oscillator to operate the device at 40Mhz
     Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
     Fosc= 8M*40/(2*2)=80Mhz for 8M input clock */
    PLLFBD = 38; /* M=40 */
    CLKDIVbits.PLLPOST = 0; /* N1=2 */
    CLKDIVbits.PLLPRE = 0; /* N2=2 */
    OSCTUN = 0; /* Tune FRC oscillator, if FRC is used */

    /* Disable Watch Dog Timer */
    RCONbits.SWDTEN = 0;

    /* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(OSCCON || 0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b011);
    // Wait for Clock switch to occur
    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1) {
    };
}

void IO_Config(void) {
    LED_Enable(LED_D1);
    LED_Enable(LED_D2);
    LED_Enable(LED_D3);
    LED_Enable(LED_D4);

    LED_Off(LED_D1);
    LED_Off(LED_D2);
    LED_Off(LED_D3);
    LED_Off(LED_D4);

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
    //RPOR4bits.RP79R = 0x0007; //RD15->SPI1:SS1
    TRISDbits.TRISD15 = 0;
    RPINR20bits.SCK1R = 0x0054; //RE4->SPI1:SCK1

    TRISDbits.TRISD14 = 0; //RD14->\LDAC

    /*ECAN1*/
    RPINR26bits.C1RXR = 108; //set CAN1 RX to RP108  (40)
    RPOR12bits.RP109R = 14; //RPOR12bits.RP109R = 14; set CAN1TX to RP109(39)
    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // lock PPS

}

void Time_Execute(void) {
    if (FLAG.Second) {
        FLAG.Second = 0;
        Update_CLOCK1();
    }
    if (T1Counter_1ms >= 20) {
        _500ms++;
        Check_Button();
#ifndef USING_SIMULATOR
        if (Device_USB == 1) {
            USBDeviceTasks();
        }
#endif
        SEVEN_SEGMENT_SCAN();
        Mission_Execute();
        Buzzer_Execute();
        if (++TDM.Job >= 5) {

            TDM.Job = 0;
#ifndef USING_SIMULATOR
            MCP4922_DualSine();
            MCP4551_Command(Volatile_Wiper_0, WriteData, VR1);
#endif

            Time_Buf_to_LINE1();
            if (++TDM.Task >= 4) {
                TDM.Task = 0;
            }
            Mission._TxJob ^= 1;
            Mission.Jobs &= 0xE0;
            ClrWdt();
        }
        if (_500ms >= 500) {
            _500ms = 0x00;
#ifndef USING_SIMULATOR
            if (Device_USB == 1) {
                /*USB*/
                if ((USBGetDeviceState() < CONFIGURED_STATE) ||
                        (USBIsDeviceSuspended() == true)) {
                    //Either the device is not configured or we are suspended
                    //  so we don't want to do execute any application code
                    return; //go back to the top of the while loop
                } else {
                    //Otherwise we are free to run user application code.
                    if (USBUSARTIsTxTrfReady() == true) {
                        sprintf(CDC_Buffer, "VR1 =0x%02X(DEC:%d),%04d mV; RC:%d \r\n",
                                VR1, VR1, ADCValues[0], CLOCK.Second);
                        putsUSBUSART(CDC_Buffer);
                    }
                }
                CDCTxService();
            }
#endif
        }
        T1Counter_1ms = 0x00;
    }
}

void Update_CLOCK1(void) {
    CLOCK.Second++;
    if (CLOCK.Second >= 60) {
        CLOCK.Second = 0;
        CLOCK.Minute++;
        MCP79410_GetTime();
        if ((CLOCK.Hour == 17 && CLOCK.Minute == 0) || (CLOCK.Hour == 11 && CLOCK.Minute == 25)) BuzzerAlarm._2 = 1;
        if (CLOCK.Minute >= 60) {
            CLOCK.Minute = 0;
            CLOCK.Hour++;
            if (CLOCK.Hour >= 24)
                CLOCK.Hour = 0;
        }
    }
}

void Time_Buf_to_LINE1(void) {
    FIRST_LINE_Data[0] = SEVEN_SEG_PATTERN[CLOCK.Hour / 10];
    FIRST_LINE_Data[1] = SEVEN_SEG_PATTERN[CLOCK.Hour % 10];
    FIRST_LINE_Data[2] = SEVEN_SEG_PATTERN[CLOCK.Minute / 10];
    FIRST_LINE_Data[3] = SEVEN_SEG_PATTERN[CLOCK.Minute % 10];
    FIRST_LINE_Data[4] = SEVEN_SEG_PATTERN[CLOCK.Second / 10];
    FIRST_LINE_Data[5] = SEVEN_SEG_PATTERN[CLOCK.Second % 10];
}

void EXT_ADC_Buf_to_LINE2(void) {
    uint16_t Temp_Value;

    Temp_Value = ADCValues[0];
    SECOND_LINE_Data[0] = SEVEN_SEG_PATTERN[Temp_Value / 1000 ];
    Temp_Value = Temp_Value - (1000 * (Temp_Value / 1000));
    SECOND_LINE_Data[1] = SEVEN_SEG_PATTERN[Temp_Value / 100];
    Temp_Value = Temp_Value - (100 * (Temp_Value / 100));
    SECOND_LINE_Data[2] = SEVEN_SEG_PATTERN[Temp_Value / 10];
    SECOND_LINE_Data[3] = SEVEN_SEG_PATTERN[Temp_Value % 10];
    SECOND_LINE_Data[4] = SEVEN_SEG_PATTERN[10];
    SECOND_LINE_Data[5] = SEVEN_SEG_PATTERN[10];
}

uint16_t MCP4551_Command(pot_memoryaddress MemoryAddress, pot_operationbits OperationBits, uint16_t Data) {
    mcp4551cmd Command = {
        .Data8 = (Data >> 8) & 0x01,
        .OperationBits = OperationBits,
        .MemoryAddress = MemoryAddress,

        .Data7_0 = Data & 0xFF
    };
    uint16_t TimeOut = 0, readData = 0;
    I2C2_TRANSACTION_REQUEST_BLOCK readTRB[2];
    if (OperationBits != ReadData) {
        if (MemoryAddress == Volatile_Wiper_1) MemoryAddress = Volatile_Wiper_0;
        I2C2_MasterWrite(Command.Byte, 2, SLAVE_I2C2_MCP4551_ADDRESS, &i2c2_msg_status);
        while (i2c2_msg_status == I2C2_MESSAGE_PENDING) {
            if (TimeOut == SLAVE_I2C2_MCP4551_DEVICE_TIMEOUT) {
                break;
            } else TimeOut++;
        }
    } else {
        // Build TRB for sending address
        I2C2_MasterWriteTRBBuild(readTRB, Command.Byte, 1, SLAVE_I2C2_MCP4551_ADDRESS);
        // Build TRB for receiving data
        I2C2_MasterReadTRBBuild(&readTRB[1], RcvData, 2, SLAVE_I2C2_MCP4551_ADDRESS);

        I2C2_MasterTRBInsert(2, readTRB, &i2c2_msg_status);
        //I2C2_MasterRead(Command.Byte, 1, RcvData, 2, SLAVE_I2C2_MCP4551_ADDRESS, &i2c2_msg_status);
        TimeOut = 0;
        while (i2c2_msg_status == I2C2_MESSAGE_PENDING) {
            if (TimeOut == SLAVE_I2C2_MCP4551_DEVICE_TIMEOUT) {
                break;
            } else TimeOut++;
        }
        if (i2c2_msg_status == I2C2_MESSAGE_COMPLETE) readData = RcvData[1] + ((RcvData[0]&0x01) << 8);
    }
    return readData;
}

void Mission_Execute(void) {
    if (TDM.Job == 0 && !Mission._0Job) { /*Freq: 200Hz*/
        if (TDM.Task == 0) { /*@0ms*/
            static unsigned CANTxTrigger = 0, CANTxContinuous = 0;
            static uint8_t _1000ms = 0;
            if (SW._TACT || SW._DIP) { /*Freq: 50Hz*/
                LED_On(LED_D4);
                if ((_SW._TACT != SW._TACT) || (_SW._DIP != SW._DIP)) CANTxTrigger = 1;
                if (_1000ms <= 50 && SW._TACT)++_1000ms;
                else if (SW._TACT == 0x00 && SW._DIP == 0x00) {
                    _1000ms = 0;
                    CANTxContinuous = 0;
                } else if (_1000ms > 50)CANTxContinuous = 1;
                if ((!C1TR23CONbits.TXREQ3 && CANTxTrigger) || CANTxContinuous) {
                    CANTxTrigger = 0;
                    FRAME_Buffer.Byte0 = CLOCK.Year;
                    FRAME_Buffer.Byte1 = CLOCK.Month;
                    FRAME_Buffer.Byte2 = CLOCK.Date;
                    FRAME_Buffer.Byte3 = CLOCK.Weekday;
                    FRAME_Buffer.Byte4 = CLOCK.Hour;
                    FRAME_Buffer.Byte5 = CLOCK.Minute;
                    FRAME_Buffer.Byte6 = CLOCK.Second;
                    FRAME_Buffer.Byte7 = 0x00;
                    Ecan1WriteTxMsgBufId(3, EXT_TRIGGER_ID, 1, 0);
                    Ecan1WriteTxMsgBufData(3, 8,
                            (uint16_t) FRAME_Buffer.DataWord0,
                            (uint16_t) FRAME_Buffer.DataWord1,
                            (uint16_t) FRAME_Buffer.DataWord2,
                            (uint16_t) FRAME_Buffer.DataWord3);
                    C1TR23CONbits.TXREQ3 = 1;
                }

            } else {
                LED_Off(LED_D4);
                CANTxTrigger = 0;
                CANTxContinuous = 0;
                _1000ms = 0;
            }
            _SW._TACT = SW._TACT;
            _SW._DIP = SW._DIP;
            Nop();
        } else if (TDM.Task == 1) { /*@5ms*/
            if (SetTime&&!SetTime1Shot) {
                MCP79410_Initialize();
                BuzzerAlarm._1 = 1;
                MCP79410_DisableOscillator();
                MCP79410_SetTime(&CurrentTime);
                MCP79410_EnableOscillator(); //Start clock by enabling oscillator
                MCP79410_GetTime();
                SetTime1Shot = 1;
            }
        } else if (TDM.Task == 2) { /*@10ms*/

        } else if (TDM.Task == 3) { /*@15ms*/

        }
        if (++_10ms >= 2) {
            _10ms = 0;
            if (!C1TR01CONbits.TXREQ0) {
                Ecan1WriteTxMsgBufId(0, EXT_20MS_ID, 1, 0);
                Ecan1WriteTxMsgBufData(0, 1,
                        (uint16_t) CLOCK.Second,
                        (uint16_t) 0x00,
                        (uint16_t) 0x00,
                        (uint16_t) 0x00);
                C1TR01CONbits.TXREQ0 = 1;
            }
        }
        Mission._0Job = 1;
    } else if (TDM.Job == 1 && !Mission._1Job) { /*Freq: 200Hz*/
        if (TDM.Task == 0) { /*@1ms*/
            static uint8_t oneshot = 0;
            if (SW.TACT._BT3) {
                if (!oneshot) BuzzerAlarm._2 = 1;
                oneshot = 1;
            } else oneshot = 0;
            if (ADCValues[0] >= 2500 || SW.TACT._BT5 || (CANidentifier == 0x41 && SW.DIP._DSW6) || bz) BuzzerFlag = 1;
            else BuzzerFlag = 0;
        } else if (TDM.Task == 1) { /*@6ms*/
            if (++_100ms >= 5) { /*Freq: 10Hz*/
                _100ms = 0;
                if (!C1TR01CONbits.TXREQ1) {
                    Ecan1WriteTxMsgBufId(1, EXT_100MS_ID, 1, 0);
                    Ecan1WriteTxMsgBufData(1, 2,
                            (uint16_t) ADCValues[0],
                            (uint16_t) 0x00,
                            (uint16_t) 0x00,
                            (uint16_t) 0x00);
                    C1TR01CONbits.TXREQ1 = 1;
                }
            }
        } else if (TDM.Task == 2) { /*@11ms*/
            static uint8_t _s500ms = 0;
            if (++_s500ms >= 25) { /*Freq: 2Hz*/
                _s500ms = 0;
                if (!C1TR23CONbits.TXREQ2) {
                    Ecan1WriteTxMsgBufId(2, EXT_500MS_ID, 1, 0);
                    Ecan1WriteTxMsgBufData(2, 8,
                            (uint16_t) 0x2301,
                            (uint16_t) 0x4523,
                            (uint16_t) 0x6745,
                            (uint16_t) 0x0167);
                    C1TR23CONbits.TXREQ2 = 1;
                }
            }
        } else if (TDM.Task == 3) { /*@16ms*/

        }

        Mission._1Job = 1;
    } else if (TDM.Job == 2 && !Mission._2Job) { /*Freq: 200Hz*/
        if (TDM.Task == 0) { /*@2ms*/

        } else if (TDM.Task == 1) { /*@7ms*/

        } else if (TDM.Task == 2) { /*@12ms*/

        } else if (TDM.Task == 3) { /*@17ms*/

        }
        Mission._2Job = 1;
    } else if (TDM.Job == 3 && !Mission._3Job) { /*Freq: 200Hz*/
        if (TDM.Task == 0) { /*@3ms*/

        } else if (TDM.Task == 1) { /*@8ms*/

        } else if (TDM.Task == 2) { /*@13ms*/

        } else if (TDM.Task == 3) { /*@18ms*/

        }
        Mission._3Job = 1;
    } else if (TDM.Job == 4 && !Mission._4Job) { /*Freq: 200Hz*/
        if (TDM.Task == 0) { /*@4ms*/

        } else if (TDM.Task == 1) { /*@9ms*/

        } else if (TDM.Task == 2) { /*@14ms*/

        } else if (TDM.Task == 3) { /*@19ms*/

        }

        if (Mission._TxJob) { /*@4ms + 10x ms*/
            EXT_ADC_Buf_to_LINE2(); /*Freq: 100Hz*/
        }

        Mission._4Job = 1;
    } else {

        TDM.Job = 0;
        TDM.Task = 0;
        Mission.Jobs = 0;
    }
}

void Check_Button(void) {
    /*BT2*/

    /*if (BT2) {
        Switch_DeBounce[BT2_Released] = 0x00;
        if (Switch_DeBounce[BT2_Pushed] < Deboune)++Switch_DeBounce[BT2_Pushed];
        else SW.TACT._BT2 = 1;
    } else {
        Switch_DeBounce[BT2_Pushed] = 0x00;
        if (Switch_DeBounce[BT2_Released] < Deboune)++Switch_DeBounce[BT2_Released];
        else SW.TACT._BT2 = 0;
    }*/
    Debounce_Execute(BT2, bt2);
    Debounce_Execute(BT3, bt3);
    Debounce_Execute(BT4, bt4);
    Debounce_Execute(BT5, bt5);
    /*DSW1*/
    /*if (DSW1) {
        Switch_DeBounce[DSW1_Off] = 0x00;
        if (Switch_DeBounce[DSW1_On] < Deboune)++Switch_DeBounce[DSW1_On];
        else SW.DIP._DSW1 = 1;
    } else {
        Switch_DeBounce[DSW1_On] = 0x00;
        if (Switch_DeBounce[DSW1_Off] < Deboune)++Switch_DeBounce[DSW1_Off];
        else SW.DIP._DSW1 = 0;
    }*/
    Debounce_Execute(DSW1, dsw1);
    Debounce_Execute(DSW2, dsw2);
    Debounce_Execute(DSW3, dsw3);
    Debounce_Execute(DSW4, dsw4);
    Debounce_Execute(DSW5, dsw5);
    Debounce_Execute(DSW6, dsw6);

}

void Debounce_Execute(uint16_t Input, DebounceSW Switch) {
    uint16_t* Output;
    uint8_t Bit_On, Bit_Off;
    if (Switch >= bt2 && Switch <= bt5) {
        Bit_On = Switch;
        Output = (uint16_t *) & SW._TACT;
    } else if (Switch >= dsw1 && Switch <= dsw6) {
        Bit_On = (Switch - dsw1);
        Output = (uint16_t *) & SW._DIP;
    }
    Bit_Off = ~(1 << Bit_On);
    Bit_On = (1 << Bit_On);
    if (Input) {
        Switch_DeBounce_Off[Switch] = 0x00;
        if (Switch_DeBounce_On[Switch] < Deboune)++Switch_DeBounce_On[Switch];
        else *Output |= Bit_On;
    } else {
        Switch_DeBounce_On[Switch] = 0x00;
        if (Switch_DeBounce_Off[Switch] < Deboune)++Switch_DeBounce_Off[Switch];
        else *Output &= Bit_Off;
    }
}

//void DebounceExec(DebounceSW Input) {
//    uint8_t Sw = 0, n = 0, N = 0xFF;
//    uint16_t* Output;
//    switch (Input) {
//        case BT2_Pushed:
//            n = 0;
//            Sw = BT2;
//            break;
//        case BT3_Pushed:
//            n = 1;
//            Sw = BT3;
//            break;
//        case BT4_Pushed:
//            n = 2;
//            Sw = BT4;
//            break;
//        case BT5_Pushed:
//            n = 3;
//            Sw = BT5;
//            break;
//        case DSW1_On:
//            n = 0;
//            Sw = DSW1;
//            break;
//        case DSW2_On:
//            n = 1;
//            Sw = DSW2;
//            break;
//        case DSW3_On:
//            n = 2;
//            Sw = DSW3;
//            break;
//        case DSW4_On:
//            n = 3;
//            Sw = DSW4;
//            break;
//        case DSW5_On:
//            n = 4;
//            Sw = DSW5;
//            break;
//        case DSW6_On:
//            n = 5;
//            Sw = DSW6;
//            break;
//        default:
//            break;
//    }
//    switch (Input) {
//        case BT5_Pushed:
//        case BT4_Pushed:
//        case BT3_Pushed:
//        case BT2_Pushed:
//            Output = (uint16_t *) &SW._TACT;
//            break;
//        case DSW6_On:
//        case DSW5_On:
//        case DSW4_On:
//        case DSW3_On:
//        case DSW2_On:
//        case DSW1_On:
//            Output = (uint16_t *) &SW._DIP;
//            break;
//        default:
//            break;
//    }
//    N &= (1 << n);
//    if (Sw) {
//        Switch_DeBounce[Input + 1] = 0x00;
//        if (Switch_DeBounce[Input] < Deboune)++Switch_DeBounce[Input];
//        else *Output |= (1 << n);
//    } else {
//        Switch_DeBounce[Input] = 0x00;
//        if (Switch_DeBounce[Input + 1] < Deboune)++Switch_DeBounce[Input + 1];
//
//        else *Output &= ~N;
//    }
//}

void Buzzer_Execute(void) {
    if (BuzzerAlarm._0) {
        if (++BzAlarm0 >= 100) {
            ++BzTrigger0;
            BzAlarm0 = 0;
        }
    } else {
        BzAlarm0 = 0;
        BzTrigger0 = 0;
    }
    if (BzTrigger0 >= 5) BuzzerAlarm._0 = 0;
    /**************************************/

    if (BuzzerAlarm._1) {
        if (++BzAlarm1 >= 200) {
            ++BzTrigger1;
            BzAlarm1 = 0;
        }
    } else {
        BzAlarm1 = 0;
        BzTrigger1 = 0;
    }
    if (BzTrigger1 >= 2) BuzzerAlarm._1 = 0;
    if (BuzzerAlarm._2) {
        if (++BzAlarm2 >= 100) {
            ++BzTrigger2;
            BzAlarm2 = 0;
        }
    } else {
        BzAlarm2 = 0;
        BzTrigger2 = 0;
    }
    if (BzTrigger2 >= 29) BuzzerAlarm._2 = 0;
    /**************************************/

    if ((BzAlarm0 < 50 && BuzzerAlarm._0) ||
            (BzAlarm1 < 150 && BuzzerAlarm._1) ||
            (BzAlarm2 < Music_Beep[BzTrigger2] && BuzzerAlarm._2)) {
        bz = 1;
    } else {

        bz = 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS2bits.C1IF = 0; // clear interrupt flag
    if (C1INTFbits.TBIF) {
        C1INTFbits.TBIF = 0;
    }

    if (C1INTFbits.RBIF) {
        C1INTFbits.RBIF = 0;
        if (C1RXFUL1bits.RXFUL4) {
            FRAME_Buffer.SID = (ecan1msgBuf[4][0] & 0x1ffc) >> 2;
            FRAME_Buffer.EID = ((uint32_t) (ecan1msgBuf[4][1] & 0x0fff) << 6) + ((ecan1msgBuf[4][2]& 0xfc00) >> 10);
            FRAME_Buffer.IDE_BIT = ecan1msgBuf[4][0] & 0x0001;
            FRAME_Buffer.DLC = ecan1msgBuf[4][2] & 0x000f;
            FRAME_Buffer.DataWord0 = ecan1msgBuf[4][3];
            FRAME_Buffer.DataWord1 = ecan1msgBuf[4][4];
            FRAME_Buffer.DataWord2 = ecan1msgBuf[4][5];
            FRAME_Buffer.DataWord3 = ecan1msgBuf[4][6];
            LED_Toggle(LED_D2);
            LED_Off(LED_D3);
            C1RXFUL1bits.RXFUL4 = 0;
        }
        if (C1RXFUL1bits.RXFUL6) {
            FRAME_Buffer.SID = (ecan1msgBuf[6][0] & 0x1ffc) >> 2;
            FRAME_Buffer.EID = ((uint32_t) (ecan1msgBuf[6][1] & 0x0fff) << 6) + ((ecan1msgBuf[6][2]& 0xfc00) >> 10);
            FRAME_Buffer.IDE_BIT = ecan1msgBuf[6][0] & 0x0001;
            FRAME_Buffer.DLC = ecan1msgBuf[6][2] & 0x000f;
            FRAME_Buffer.DataWord0 = ecan1msgBuf[6][3];
            FRAME_Buffer.DataWord1 = ecan1msgBuf[6][4];
            FRAME_Buffer.DataWord2 = ecan1msgBuf[6][5];
            FRAME_Buffer.DataWord3 = ecan1msgBuf[6][6];
            CurrentTime.year = FRAME_Buffer.Byte0;
            CurrentTime.month = FRAME_Buffer.Byte1;
            CurrentTime.date = FRAME_Buffer.Byte2;
            CurrentTime.weekday = FRAME_Buffer.Byte3;
            CurrentTime.hour = FRAME_Buffer.Byte4;
            CurrentTime.min = FRAME_Buffer.Byte5;
            CurrentTime.sec = FRAME_Buffer.Byte6;
            LED_Toggle(LED_D3);
            if (FRAME_Buffer.Byte7 != 0) SetTime = 1;
            C1RXFUL1bits.RXFUL6 = 0;
        }
        if (C1RXFUL1bits.RXFUL7) {
            FRAME_Buffer.SID = (ecan1msgBuf[7][0] & 0x1ffc) >> 2;
            FRAME_Buffer.EID = ((uint32_t) (ecan1msgBuf[7][1] & 0x0fff) << 6) + ((ecan1msgBuf[7][2]& 0xfc00) >> 10);
            if (FRAME_Buffer.EID & 0x01) LED_Toggle(LED_D3);
            LED_Off(LED_D2);
            C1RXFUL1bits.RXFUL7 = 0;
        }
        CANidentifier = ((uint32_t) FRAME_Buffer.SID << 18) + FRAME_Buffer.EID;
        Nop();
    }
}

uint8_t I2C_Check_Device(uint8_t ADDRESS, i2c_modules MODULES) {
    uint16_t timeout = 0;
    if (MODULES == I2C1) {
        I2C1_MasterWrite(0x00, 0x00, ADDRESS, &i2c1_msg_status);
        while (i2c1_msg_status == I2C1_MESSAGE_PENDING) {
            if (timeout == MAX_TRY) {
                return 0;
            } else {
                ++timeout;
            }
        }
        if (i2c1_msg_status == I2C1_MESSAGE_COMPLETE) {
            return 1;
        } else {
            return 0;
        }
    } else if (MODULES == I2C2) {
        I2C2_MasterWrite(0x00, 0x00, ADDRESS, &i2c2_msg_status);
        while (i2c2_msg_status == I2C2_MESSAGE_PENDING) {
            if (timeout == MAX_TRY) {
                return 0;
            } else {
                ++timeout;
            }
        }
        if (i2c2_msg_status == I2C2_MESSAGE_COMPLETE) {
            return 1;
        } else {
            return 0;
        }
    }
    return 0;
}

void initial_LCD(void) {
    I2C_LCD.BL = I2C_STATUS.Enale = 1;
    if (I2C_STATUS.Enale) {
        I2C_STATUS._4bit = 1;
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

        I2C_STATUS._4bit = 0;
    }
    LCD_WriteInstruction(FOUR_BIT);
#ifndef USING_SIMULATOR
    __delay_us(100); /*wait time > 100us*/
#endif

    LCD_WriteInstruction(CURSOR_ON);
#ifndef USING_SIMULATOR
    __delay_us(1800); /*wait time > 1.6ms*/
#endif

}

void LCD_WriteInstruction(uint8_t Instruction) {
    uint8_t LCD_Instruction[4];
    uint8_t *Temporary = LCD_Instruction;
    uint8_t length;
    uint16_t TimeOut;
    I2C_LCD.RS = 0;
    I2C_LCD.RW = 0;
    I2C_ADRRESS.Address = SLAVE_I2C_LCD_ADDRESS;
    I2C_ADRRESS.R_nW = 0;
    length = 2;
    if (!I2C_STATUS._4bit) {
        I2C_LCD.D7_D4 = ((Instruction & 0xF0) >> 4) &0x0F;
        I2C_LCD.EN = 1;
        *Temporary++ = I2C_LCD.Byte;
        I2C_LCD.EN = 0;
        *Temporary++ = I2C_LCD.Byte;
        length = 4;

    }
    I2C_LCD.D7_D4 = (Instruction & 0x0F);
    I2C_LCD.EN = 1;
    *Temporary++ = I2C_LCD.Byte;
    I2C_LCD.EN = 0;
    *Temporary++ = I2C_LCD.Byte;
    I2C2_MasterWrite(LCD_Instruction, length, I2C_ADRRESS.Address, &i2c2_msg_status);
    TimeOut = 0;
#ifndef USING_SIMULATOR
    while (i2c2_msg_status == I2C2_MESSAGE_PENDING) {
        // add some delay here

        // timeout checking
        // check for max retry and skip this byte
        if (TimeOut == 1500) {
            break;
        } else TimeOut++;
    }
#endif
}

void LCD_WriteData(uint8_t DATA) {
    uint8_t LCD_Data[4];
    uint8_t *Temporary = LCD_Data;
    uint8_t length;
    uint16_t slaveTimeOut;
    I2C_LCD.RS = 1;
    I2C_LCD.RW = 0;
    I2C_ADRRESS.Address = SLAVE_I2C_LCD_ADDRESS;
    I2C_ADRRESS.R_nW = 0;
    length = 4;
    I2C_LCD.D7_D4 = ((DATA & 0xF0) >> 4) &0x0F;
    I2C_LCD.EN = 1;
    *Temporary++ = I2C_LCD.Byte;
    I2C_LCD.EN = 0;
    *Temporary++ = I2C_LCD.Byte;

    I2C_LCD.D7_D4 = (DATA & 0x0F);
    I2C_LCD.EN = 1;
    *Temporary++ = I2C_LCD.Byte;
    I2C_LCD.EN = 0;
    *Temporary++ = I2C_LCD.Byte;
    I2C2_MasterWrite(LCD_Data, length, I2C_ADRRESS.Address, &i2c2_msg_status);
    slaveTimeOut = 0;
#ifndef USING_SIMULATOR
    while (i2c2_msg_status == I2C2_MESSAGE_PENDING) {
        // add some delay here

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == 1500) {
            break;
        } else slaveTimeOut++;
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

void LCD_Set_Cursor(uint8_t CurY, uint8_t CurX) {
    LCD_WriteInstruction(0x80 + CurY * 0x40 + CurX);
#ifndef USING_SIMULATOR
    __delay_us(50);
#endif
}
