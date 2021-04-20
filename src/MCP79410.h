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

#ifndef MCP79410_H
#define MCP79410_H

#include "Common.h"
#define CONFIG_TR_QUEUE_LENGTH 16

/*i2c*/
typedef enum {
    Write,
    Read
} I2CnW_R;

typedef enum {
    /*Section 5.3 “Timekeeping”*/
    RTCSEC, // Register Address: Time Second
    RTCMIN, // Register Address: Time Minute
    RTCHOUR, // Register Address: Time Hour
    RTCWKDAY, // Register Address: Date Day of Week
    RTCDATE, // Register Address: Date Day
    RTCMTH, // Register Address: Date Month
    RTCYEAR, // Register Address: Date Year
    CONTROL,
    OSCTRIM,
    EEUNLOCK,
    /*Section 5.4 “Alarms”*/
    ALM0SEC,
    ALM0MIN,
    ALM0HOUR,
    ALM0WKDAY,
    ALM0DATE,
    ALM0MTH,
    //\Reserved 10h
    ALM1SEC = 0x11,
    ALM1MIN,
    ALM1HOUR,
    ALM1WKDAY,
    ALM1DATE,
    ALM1MTH,
    //\Reserved 17h
    /*Section 5.7.1 “Power-Fail Time-Stamp”*/
    /*Power-Down Time Stamp*/
    PWRDNMIN = 0x18,
    PWRDNHOUR,
    PWRDNDATE,
    PWRDNMTH,
    /*Power-Up Time Stamp*/
    PWRUPMIN,
    PWRUPHOUR,
    PWRUPDATE,
    PWRUPMTH

} rtcc_registerbits;

/*MCP79410*/
typedef struct _RTCC_Struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t weekday;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} RTCC_Struct;

typedef enum Alarm {
    ZERO,
    ONE
} Alarm_t;

typedef enum AlarmStatus {
    NOT_SET,
    SET
} AlarmStatus_t;

typedef enum PMAM {
    AMT,
    PMT
} PMAM_t;

typedef enum Format {
    H24,
    H12
} Format_t;

typedef enum Match {
    SECONDS_MATCH,
    MINUTES_MATCH,
    HOURS_MATCH,
    WEEKDAY_MATCH,
    DATE_MATCH,
    FULL_DATE_MATCH
} Match_t;

typedef enum MFP_MODE {
    GPO,
    ALARM_INTERRUPT,
    SQUARE_WAVE
} MFP_t;

typedef enum MFP_POL {
    LOWPOL,
    HIGHPOL
} Polarity_t;

/************************GLOBAL CONSTANTS RTCC - INITIALIZATION****************/
#define TRUE    1
#define FALSE   0

#define  PM                0x20       //  post-meridian bit (HOUR)
#define  HOUR_FORMAT       0x40       //  Hour format
#define  OUT_PIN           0x80       //  = b7 (CTRL)
#define  SQWEN             0x40       //  SQWE = b6 (CTRL)
#define  ALM_NO            0x00       //  no alarm activated        (CTRL)
#define  ALM_0             0x10       //  ALARM0 is       activated (CTRL)
#define  ALM_1             0x20       //  ALARM1 is       activated (CTRL)
#define  ALM_01            0x30       //  both alarms are activated (CTRL)
#define  MFP_01H           0x00       //  MFP = SQVAW(01 HERZ)      (CTRL)
#define  MFP_04K           0x01       //  MFP = SQVAW(04 KHZ)       (CTRL)
#define  MFP_08K           0x02       //  MFP = SQVAW(08 KHZ)       (CTRL)
#define  MFP_32K           0x03       //  MFP = SQVAW(32 KHZ)       (CTRL)
#define  MFP_64H           0x04       //  MFP = SQVAW(64 HERZ)      (CTRL)
#define  ALMx_POL          0x80       //  polarity of MFP on alarm  (ALMxCTL)
#define  ALMxC_SEC         0x00       //  ALARM compare on SEC      (ALMxCTL)
#define  ALMxC_MIN         0x10       //  ALARM compare on MIN      (ALMxCTL)
#define  ALMxC_HR          0x20       //  ALARM compare on HOUR     (ALMxCTL)
#define  ALMxC_DAY         0x30       //  ALARM compare on DAY      (ALMxCTL)
#define  ALMxC_DAT         0x40       //  ALARM compare on DATE     (ALMxCTL)
#define  ALMxC_ALL         0x70       //  ALARM compare on all param(ALMxCTL)
#define  ALMx_IF           0x08       //  MASK of the ALARM_IF      (ALMxCTL)

#define  OSCRUN            0x20       //  state of the oscillator(running or not)
#define  PWRFAIL           0x10
#define  VBATEN            0x08       //  enable battery for back-up
#define  VBAT_DIS          0x37       //  disable battery back-up
#define  START_32KHZ       0x80       //  start crystal: ST = b7 (SEC)
#define  LP                0x20       //  mask for the leap year bit(MONTH REG)
#define  HOUR_12           0x40       //  12 hours format   (HOUR)

#define  LPYR              0x20

#define ALM1MSK2           0x40
#define ALM1MSK1           0x20
#define ALM1MSK0           0x10

#define ALM0MSK2           0x40
#define ALM0MSK1           0x20
#define ALM0MSK0           0x10

void MCP79410_Initialize(void);
void MCP79410_EnableOscillator(void);
void MCP79410_DisableOscillator(void);
uint8_t MCP79410_IsRunning(void);

void MCP79410_GetTime(void);
void MCP79410_SetTime(RTCC_Struct *time);
void MCP79410_SetHourFormat(Format_t format);
void MCP79410_SetPMAM(PMAM_t meridian);

void MCP79410_EnableAlarm(Alarm_t alarm);
void MCP79410_DisableAlarm(Alarm_t alarm);
AlarmStatus_t MCP79410_GetAlarmStatus(Alarm_t alarm);
void MCP79410_ClearInterruptFlag(Alarm_t alarm);
void MCP79410_SetAlarmTime(RTCC_Struct *time, Alarm_t alarm);
void MCP79410_SetAlarmMFPPolarity(Polarity_t MFP_pol, Alarm_t alarm);
void MCP79410_SetAlarmMatch(Match_t match, Alarm_t alarm);
void MCP79410_SetMFP_Functionality(MFP_t mode);
void MCP79410_SetMFP_GPOStatus(Polarity_t status);

uint8_t MCP79410_CheckPowerFailure(void);
uint8_t MCP79410_IsVbatEnabled(void);
void MCP79410_EnableVbat(void);
void MCP79410_DisableVbat(void);
RTCC_Struct* MCP79410_GetPowerUpTime(void);
RTCC_Struct* MCP79410_GetPowerDownTime(void);

uint8_t MCP79410_dec2bcd(uint8_t num);
uint8_t MCP79410_bcd2dec(uint8_t num);
uint16_t MCP79410_Command(rtcc_registerbits MemoryAddress, uint8_t data, I2CnW_R nW_R);

void MCP79410_EnableVbat(void);

#endif //MCP79410_H
