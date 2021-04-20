/* 
 * File:   Common.h
 * Author: user
 *
 * Created on 2020年4月29日, 下午 4:53
 */

#ifndef COMMON_H
#define	COMMON_H

#ifdef	__cplusplus
extern "C" {
#endif
#define FOSC    (80000000ULL)
#define FCY     (FOSC/2) 
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LCM.h"
#include "SEVEN_SEG.h"
#include "Timers.h"
#include "adc1.h"
#include "UART1.h"
#include "i2c1.h"
#include "i2c2.h"
#include "spi1.h"
#include "../Include_Files/LIB_USB_CDC_MU810.h"
#include "MP_Car_leds.h"
#include "ecan1_config.h"
#include "ecan1drv.h"
#include "pwm.h"
#include "time.h"
#include "MCP79410.h"

    /* check if build is for a real debug tool */
#ifdef __MPLAB_DEBUGGER_SIMULATOR
#warning Debug with broken MPLAB simulator
#define USING_SIMULATOR
#endif

#define UART1_BAUDRATE 115200
#define UART1_BRP_VAL ((FCY/UART1_BAUDRATE)/4)-1
    /* CAN1 Baud Rate Configuration         */
#define CAN1_BITRATE 500000
#define NTQ     20  // 20 Time Quanta in a Bit Time
#define CAN1_BRP_VAL ( (FCY / (2 * NTQ * CAN1_BITRATE)) - 1 )
#define I2C1_FSCL 400000
#define I2C2_FSCL 100000
#define I2C1_BRG_VAL (FCY/I2C1_FSCL-130*FCY/1000000000L)-1
#define I2C2_BRG_VAL (FCY/I2C2_FSCL-130*FCY/1000000000L)-1

    /*Vref+ = Vdd = 3.3V : 4095*/
    //#define adc2volt_3_3V(ADC_Value)    (ADC_Value*825L+512)/1024

    /*Vref+ = Vdd = 3.3V : 1023*/
#define adc2volt_3_3V(ADC_Value)    ADC_Value*4-(ADC_Value*3171L+2048)/4096

    /*LED*/
#define LED1 LATAbits.LATA5
#define LED2 LATAbits.LATA4
#define LED3 LATAbits.LATA3
#define LED4 LATAbits.LATA2

#define DIR_LED1 TRISAbits.TRISA5
#define DIR_LED2 TRISAbits.TRISA4
#define DIR_LED3 TRISAbits.TRISA3
#define DIR_LED4 TRISAbits.TRISA2

    /*Buzzer*/
#define Buzzer LATGbits.LATG13

    /*MCP4922*/
#define nLDAC LATDbits.LATD14
#define nSS LATDbits.LATD15

    /*UART1*/
#define UART1_BUFFER_SIZE 24

    /*I2C*/
#define MAX_TRY 500
#define SLAVE_I2C_RPB1600_ADDRESS 0x47
#define SLAVE_I2C_LCD_ADDRESS 0x27
    /*RTCC Register/SRAM Control Byte*/
#define SLAVE_I2C1_MCP79410_REG_ADDRESS 0x6F /*MCP79410 ‘1101 111’b*/
    /*EEPROM Control Byte*/
#define SLAVE_I2C1_MCP79410_EEPROM_ADDRESS 0x57 /*MCP79410 ‘1010 111’b*/
#define SLAVE_I2C2_MCP4551_ADDRESS 0x2E /*MCP45X1 ‘0101 11’b + A0*/

    /*I2C1*/
#define SLAVE_I2C1_DEVICE_TIMEOUT 1500
#define SLAVE_I2C1_MCP79410_DEVICE_TIMEOUT 1500   // define slave timeout 
#define MCP79410_RETRY_MAX 500

    /*I2C2*/
#define SLAVE_I2C2_DEVICE_TIMEOUT 1500
    /*MCP4551-103E/XX: 10 kOhm, 8-LD Device*/
#define SLAVE_I2C2_MCP4551_DEVICE_TIMEOUT 1500   // define slave timeout 

    /*ECAN1*/
#define EXT_20MS_ID  0x400
#define EXT_100MS_ID  0x405
#define EXT_500MS_ID    0x200
#define EXT_TRIGGER_ID    0x18ABCDEF
#define RCVx_ID 0x40
#define RCVy_ID 0x30
#define ECAN1_MSG_BUF_LENGTH 32
    typedef uint16_t ECAN1MSGBUF[ECAN1_MSG_BUF_LENGTH][8];
    __eds__ extern ECAN1MSGBUF ecan1msgBuf
    __attribute__((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));

    extern struct _switch {

        union {

            struct {
                unsigned _BT2 : 1;
                unsigned _BT3 : 1;
                unsigned _BT4 : 1;
                unsigned _BT5 : 1;
            } TACT;
            uint8_t _TACT;
        };

        union {

            struct {
                unsigned _DSW1 : 1;
                unsigned _DSW2 : 1;
                unsigned _DSW3 : 1;
                unsigned _DSW4 : 1;
                unsigned _DSW5 : 1;
                unsigned _DSW6 : 1;
            } DIP;
            uint8_t _DIP;
        };
    } SW, _SW;

    extern struct _tdm {
        unsigned Job : 3;
        unsigned Task : 3;
    } TDM; // Time-Division Multiplexing

    extern union _mission {

        struct {
            unsigned _0Job : 1;
            unsigned _1Job : 1;
            unsigned _2Job : 1;
            unsigned _3Job : 1;
            unsigned _4Job : 1;
            unsigned _TxJob : 1;
            unsigned : 2;
        };
        uint8_t Jobs;
    } Mission; // Time-Division Multiplexing

    extern struct _flag {
        unsigned Second : 1;
    } FLAG;

    extern struct _buzzeralarm {
        unsigned _0 : 1; //on 50ms off 50ms 5shot
        unsigned _1 : 1; //on 150ms off 50ms 2shot
        unsigned _2 : 1;
        unsigned _3 : 1;
        unsigned _4 : 1;
        unsigned Oneshot : 1;
        unsigned : 2;
    } BuzzerAlarm;

    extern struct _clock {
        uint8_t Year;
        uint8_t Month;
        uint8_t Date;
        uint8_t Weekday;
        uint8_t Hour;
        uint8_t Minute;
        uint8_t Second;
    } CLOCK;

    extern struct _frame_buffer { /*The argument to the aligned attribute must be a power of two.*/

        union {

            struct {
                uint16_t SID;
                uint32_t EID;
                uint8_t DLC;
                uint8_t IDE_BIT;
                uint8_t DATA_IN;
            };

            struct {
                uint16_t Null1;
                uint32_t Null2;
                uint8_t Null3;
                uint8_t Null4;
                uint8_t Null5;
            };
        };

        union {

            struct {
                uint16_t DataWord0;
                uint16_t DataWord1;
                uint16_t DataWord2;
                uint16_t DataWord3;

            };

            struct {
                uint8_t Byte0;
                uint8_t Byte1;
                uint8_t Byte2;
                uint8_t Byte3;
                uint8_t Byte4;
                uint8_t Byte5;
                uint8_t Byte6;
                uint8_t Byte7;
            };
        };
    } FRAME_Buffer;

    extern struct _i2c_status {
        unsigned Enale : 1;
        unsigned _4bit : 1;
    } I2C_STATUS;

    extern union _i2c_lcd {

        struct {
            unsigned RS : 1; //p0
            unsigned RW : 1; //p1
            unsigned EN : 1; //p2
            unsigned BL : 1; //p3
            unsigned D4 : 1; //p4
            unsigned D5 : 1; //p5
            unsigned D6 : 1; //p6
            unsigned D7 : 1; //p7
        };

        struct {
            unsigned : 4;
            unsigned D7_D4 : 4;
        };
        uint8_t Byte;
    } I2C_LCD;

    extern union _i2c_address {

        struct {
            unsigned R_nW : 1;
            unsigned Address : 7;
        };
        uint8_t Address_R_nW;
    } I2C_ADRRESS;

    extern union _i2c_device {

        struct {
            unsigned LCD : 1;
            unsigned RPB1600 : 1;
            unsigned MCP4551 : 1;
            unsigned MCP79410 : 1;
            unsigned Check : 1;
        };
        unsigned char ALL;
    } I2C_Device;

    typedef enum {
        I2C1 = 1,
        I2C2 = 2
    } i2c_modules;

    /*MCP4551*/
    typedef enum {
        WriteData,
        Increment,
        Decrement,
        ReadData
    } pot_operationbits;

    typedef enum {
        Volatile_Wiper_0,
        Volatile_Wiper_1,
        Volatile_TCON_Register = 0x04
    } pot_memoryaddress;

    typedef union {

        struct {
            unsigned Data8 : 1;
            unsigned : 1;
            pot_operationbits OperationBits : 2;
            pot_memoryaddress MemoryAddress : 4;

            uint8_t Data7_0;
        };
        uint8_t Byte[2];
    } mcp4551cmd;

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

    /*Debounce*/
    typedef enum {
        bt2,
        bt3,
        bt4,
        bt5,
        dsw1,
        dsw2,
        dsw3,
        dsw4,
        dsw5,
        dsw6,
    } DebounceSW;



    void DMA0Init(void); //UART1 transmitter
    void DMA1Init(void); //UART1 receiver
    void DMA2Init(void); //ADC1 convert done
    void DMA3Init(void); //SPI Transmission
    void DMA4Init(void); //SPI Reception
    void DMA5Init(void); //ECAN1 Transmission
    void DMA6Init(void); //ECAN1 Reception

    uint16_t MCP4551_Command(pot_memoryaddress MemoryAddress, pot_operationbits OperationBits, uint16_t Data);

    void MCP4922_DualSine(void);


#ifdef	__cplusplus
}
#endif

#endif	/* COMMON_H */

