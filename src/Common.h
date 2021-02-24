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

    //#define __DEBUG_RTCC__


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

    /*I2C1*/
#define SLAVE_I2C1_DEVICE_TIMEOUT 1500
#define SLAVE_I2C1_MCP79410_DEVICE_TIMEOUT 1500   // define slave timeout 
#define MCP79410_RETRY_MAX 500
    /*RTCC Register/SRAM Control Byte*/
#define SLAVE_I2C1_MCP79410_REG_ADDRESS 0x6F /*MCP79410 ‘1101 111’b*/
    /*EEPROM Control Byte*/
#define SLAVE_I2C1_MCP79410_EEPROM_ADDRESS 0x57 /*MCP79410 ‘1010 111’b*/

    /*I2C2*/
#define SLAVE_I2C2_DEVICE_TIMEOUT 1500
    /*MCP4551-103E/XX: 10 kOhm, 8-LD Device*/
#define SLAVE_I2C2_MCP4551_DEVICE_TIMEOUT 1500   // define slave timeout 
#define SLAVE_I2C2_MCP4551_ADDRESS 0x2E /*MCP45X1 ‘0101 11’b + A0*/

    /*ECAN1*/
#define EXT_20MS_ID  0x400
#define EXT_100MS_ID  0x405
#define EXT_500MS_ID    0x200
#define EXT_TRIGGER_ID    0x666
#define RCVx_ID    0x40
#define RCVy_ID    0x30
#define ECAN1_MSG_BUF_LENGTH    32
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
            unsigned char _TACT;
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
            unsigned char _DIP;
        };
    } SW, _SW;

    extern struct _tdm {
        unsigned Job : 3;
        unsigned Task : 2;
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
        unsigned char Jobs;
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
        unsigned char Year;
        unsigned char Month;
        unsigned char Date;
        unsigned char Weekday;
        unsigned char Hour;
        unsigned char Minute;
        unsigned char Second;
    } CLOCK;

    extern union _frame_buffer {

        struct {
            unsigned int SID;
            unsigned long EID;
            unsigned char DLC;
            unsigned char IDE_BIT;
            unsigned char DATA_IN;
            unsigned int DataWord0;
            unsigned int DataWord1;
            unsigned int DataWord2;
            unsigned int DataWord3;
        };

        struct {
            unsigned int Null1;
            unsigned long Null2;
            unsigned char Null3;
            unsigned char Null4;
            unsigned char Null5;
            unsigned char Byte0;
            unsigned char Byte1;
            unsigned char Byte2;
            unsigned char Byte3;
            unsigned char Byte4;
            unsigned char Byte5;
            unsigned char Byte6;
            unsigned char Byte7;
        };
    } FRAME_Buffer;

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

            unsigned char Data7_0;
        };
        unsigned char Byte[2];
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
        unsigned int Int;
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
    void DELAY_US(unsigned int DELAY);

    uint16_t MCP4551_Command(pot_memoryaddress MemoryAddress, pot_operationbits OperationBits, uint16_t Data);

    void MCP4922_DualSine(void);


#ifdef	__cplusplus
}
#endif

#endif	/* COMMON_H */

