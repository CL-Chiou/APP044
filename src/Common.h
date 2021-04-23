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
#include <stdint.h>
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
#include "MCP4551.h"
#include "MCP4922.h"

    /* check if build is for a real debug tool */
#ifdef __MPLAB_DEBUGGER_SIMULATOR
#define USING_SIMULATOR
#endif

    /*UART1*/
#define U1TX_MSG_BUF_LENGTH 24
#define U1BAUDRATE 115200
#define U1BRP_VAL ((FCY/U1BAUDRATE)/4)-1
    /* CAN1 Baud Rate Configuration */
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
#define adc2volt_3_3V(adcvalue)    adcvalue*4-(adcvalue*3171L+2048)/4096  

    /*LED*/
#define LED1 LATAbits.LATA5
#define LED2 LATAbits.LATA4
#define LED3 LATAbits.LATA3
#define LED4 LATAbits.LATA2

#define DIR_LED1 TRISAbits.TRISA5
#define DIR_LED2 TRISAbits.TRISA4
#define DIR_LED3 TRISAbits.TRISA3
#define DIR_LED4 TRISAbits.TRISA2

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

    /*Buzzer*/
#define Buzzer LATGbits.LATG13

    /*MCP4922*/
#define nLDAC LATDbits.LATD14
#define nSS LATDbits.LATD15

    /* Display ON/OFF Control defines */
#define LCD_DON                     0b00001111  /* Display on      */
#define LCD_DOFF                    0b00001011  /* Display off     */
#define LCD_CURSOR_ON               0b00001111  /* Cursor on       */
#define LCD_CURSOR_OFF              0b00001101  /* Cursor off      */
#define LCD_BLINK_ON                0b00001111  /* Cursor Blink    */
#define LCD_BLINK_OFF               0b00001110  /* Cursor No Blink */
#define LCD_BLINK_OFF_CURSOR_OFF    0b00001100  /* Cursor No Blink, Cursor off */
#define LCD_RETURN_HOME             0b00000001  /* Sets DDRAM address 0 in address counter */

    /* Cursor or Display Shift defines */
#define LCD_SHIFT_CUR_LEFT          0b00000100  /* Cursor shifts to the left   */
#define LCD_SHIFT_CUR_RIGHT         0b00000101  /* Cursor shifts to the right  */
#define LCD_SHIFT_DISP_LEFT         0b00000110  /* Display shifts to the left  */
#define LCD_SHIFT_DISP_RIGHT        0b00000111  /* Display shifts to the right */

    /* Function Set defines */
#define LCD_FOUR_BIT                0b00101100  /* 4-bit Interface               */
#define LCD_EIGHT_BIT               0b00111100  /* 8-bit Interface               */
#define LCD_LINE_5X7                0b00110000  /* 5x7 characters, single line   */
#define LCD_LINE_5X10               0b00110100  /* 5x10 characters               */
#define LCD_LINES_5X7               0b00111000  /* 5x7 characters, multiple line */

    /*I2C Address*/
#define I2C_MAX_TRY 500
#define SLAVE_I2C_RPB1600_ADDRESS 0x47
#define SLAVE_I2C_LCD_ADDRESS 0x27
#define SLAVE_I2C1_MCP79410_REG_ADDRESS 0x6F /*MCP79410 ‘1101 111’b </RTCC Register/SRAM Control Byte>*/  
#define SLAVE_I2C1_MCP79410_EEPROM_ADDRESS 0x57 /*MCP79410 ‘1010 111’b </EEPROM Control Byte>*/
#define SLAVE_I2C2_MCP4551_ADDRESS 0x2E /*MCP45X1 ‘0101 11’b + A0*/

    /*I2C1*/
#define SLAVE_I2C1_DEVICE_TIMEOUT 1500

    /*I2C2*/
#define SLAVE_I2C2_DEVICE_TIMEOUT 1500
    /*Single Nonvolatile 8-bit Potentiometer*/

    /*ECAN1*/
#define CANTX_ID_20MS 0x400
#define CANTX_ID_100MS 0x405
#define CANTX_ID_500MS 0x200
#define CANTX_ID_TRIGGER 0x18ABCDEF
#define CANRX_ID_1 0x40
#define CANRX_ID_2 0x30
#define ECAN1_MSG_BUF_LENGTH 32
    typedef uint16_t ECAN1MessageBuffer_t[ECAN1_MSG_BUF_LENGTH][8];
    __eds__ extern ECAN1MessageBuffer_t ecan1msgBuf
    __attribute__((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));

    typedef struct xSwitchItem {

        union {

            struct {
                unsigned _BT2 : 1;
                unsigned _BT3 : 1;
                unsigned _BT4 : 1;
                unsigned _BT5 : 1;
            } TACT;
            uint8_t bTACT;
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
            uint8_t bDIP;
        };
    } SwitchItem_t;

    typedef struct xTimeFlag {
        unsigned Second : 1;
    } TimeFlag_t;

    typedef struct xBuzzerMode {
        unsigned _0 : 1; //on 50ms off 50ms 5shot
        unsigned _1 : 1; //on 150ms off 50ms 2shot
        unsigned _2 : 1;
        unsigned _3 : 1;
        unsigned _4 : 1;
        unsigned : 3;
    } BuzzerMode_t;

    typedef struct xRealTimeClock {
        uint8_t Year;
        uint8_t Month;
        uint8_t Date;
        uint8_t Weekday;
        uint8_t Hour;
        uint8_t Minute;
        uint8_t Second;
    } RealTimeClock_t;

    typedef struct xCANDataFrame { /*The argument to the aligned attribute must be a power of two.*/

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
                uint16_t wordData0;
                uint16_t wordData1;
                uint16_t wordData2;
                uint16_t wordData3;

            };

            struct {
                uint8_t byteData0;
                uint8_t byteData1;
                uint8_t byteData2;
                uint8_t byteData3;
                uint8_t byteData4;
                uint8_t byteData5;
                uint8_t byteData6;
                uint8_t byteData7;
            };
        };
    } CANDataFrame_t;

    typedef struct xI2CLCDFlag {
        unsigned Enale : 1;
        unsigned n8bit_4bit : 1;
    } I2CLCDFlag_t;

    typedef union xI2CLCDIO {

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
        uint8_t byteData;
    } I2CLCDIO_t;

    typedef union xI2CLCDAddress {

        struct {
            unsigned R_nW : 1;
            unsigned Address : 7;
        };
        uint8_t byteData;
    } I2CLCDAddress_t;

    typedef union xI2CDevice {

        struct {
            unsigned LCD : 1;
            unsigned RPB1600 : 1;
            unsigned MCP4551 : 1;
            unsigned MCP79410 : 1;
            unsigned Check : 1;
        };
        uint8_t ALL;
    } I2CDevice_t;

    typedef enum {
        I2C1 = 1,
        I2C2 = 2
    } I2CModules_t;

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
    } DebounceSwitch_t;

    void DMA0_Initialize(void); //UART1 transmitter
    void DMA1_Initialize(void); //UART1 receiver
    void DMA2_Initialize(void); //ADC1 convert done
    void DMA3_Initialize(void); //SPI Transmission
    void DMA4_Initialize(void); //SPI Reception
    void DMA5_Initialize(void); //ECAN1 Transmission
    void DMA6_Initialize(void); //ECAN1 Reception
    void MultiTask(void);

#ifdef	__cplusplus
}
#endif

#endif	/* COMMON_H */

