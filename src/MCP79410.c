/*
 * File:   MCP79410.c
 * Author: user
 *
 * Created on December 21, 2020, 9:01 AM
 */

#include "Common.h"
#include "MCP79410.h"
#include "i2c1.h"
#include "time.h"

extern xRealTimeClock_t SystemClock;
extern I2C1_MESSAGE_STATUS I2C1MessageStatus;
extern RTCC_t Now;

void MCP79410_Initialize(void) {
    MCP79410_SetHourFormat(H24); //Set hour format to military time standard
    MCP79410_EnableVbat(); //Enable battery backup
    MCP79410_EnableOscillator(); //Start clock by enabling oscillator

}

void MCP79410_EnableOscillator(void) {
    uint8_t OSCRUN_bit = MCP79410_Command(RTCWKDAY, 0x00, Read); //Read day + OSCON bit
    OSCRUN_bit = OSCRUN_bit | OSCRUN;
    MCP79410_Command(RTCWKDAY, OSCRUN_bit, Write);
}

void MCP79410_DisableOscillator(void) {
    uint8_t OSCRUN_bit = MCP79410_Command(RTCWKDAY, 0x00, Read); //Read day + OSCON bit
    OSCRUN_bit = OSCRUN_bit & ~OSCRUN;
    MCP79410_Command(RTCWKDAY, OSCRUN_bit, Write);
}

uint8_t MCP79410_IsRunning(void) {
    uint8_t mask = MCP79410_Command(RTCWKDAY, 0x00, Read);

    if ((mask & OSCRUN) == OSCRUN) //If oscillator = already running, do nothing.
    {
        return TRUE;
    } else {
        return FALSE;
    }
}

void MCP79410_GetTime(void) {
    Now.sec = MCP79410_bcd2dec(MCP79410_Command(RTCSEC, 0x00, Read) & 0x7F);
    Now.min = MCP79410_bcd2dec(MCP79410_Command(RTCMIN, 0x00, Read));

    uint8_t hour_t = MCP79410_Command(RTCHOUR, 0x00, Read);

    hour_t = ((hour_t & HOUR_12) == HOUR_12) ? (hour_t & 0x1F) : (hour_t & 0x3F); //hour is in 24 hour format

    Now.hour = MCP79410_bcd2dec(hour_t);
    Now.weekday = MCP79410_bcd2dec(MCP79410_Command(RTCWKDAY, 0x00, Read) & ~(OSCRUN | PWRFAIL | VBATEN));
    Now.date = MCP79410_bcd2dec(MCP79410_Command(RTCDATE, 0x00, Read));
    Now.month = MCP79410_bcd2dec(MCP79410_Command(RTCMTH, 0x00, Read) & ~(LPYR));
    Now.year = MCP79410_bcd2dec(MCP79410_Command(RTCYEAR, 0x00, Read));

    SystemClock.Year = Now.year;
    SystemClock.Month = Now.month;
    SystemClock.Date = Now.date;
    SystemClock.Weekday = Now.weekday;
    SystemClock.Hour = Now.hour;
    SystemClock.Minute = Now.min;
    SystemClock.Second = Now.sec;
    Nop();

}

void MCP79410_SetTime(RTCC_t *time) {
    uint8_t sec = MCP79410_Command(RTCSEC, 0x00, Read); //Seconds
    uint8_t min = 0; //Minutes
    uint8_t hour = MCP79410_Command(RTCHOUR, 0x00, Read); //Hours
    uint8_t weekday = MCP79410_Command(RTCWKDAY, 0x00, Read); //Weekday
    uint8_t date = 0; //Date
    uint8_t month = MCP79410_Command(RTCMTH, 0x00, Read); //Month
    uint8_t year = 0;
    if ((sec & START_32KHZ) == START_32KHZ) //Seconds register
    {
        sec = MCP79410_dec2bcd(time->sec) | START_32KHZ;
    } else {
        sec = MCP79410_dec2bcd(time->sec);
    }

    min = MCP79410_dec2bcd(time->min); //Minutes

    if ((hour & HOUR_12) == HOUR_12) //Hour register
    {
        hour = MCP79410_dec2bcd(time->hour) | HOUR_12;
    } else {
        hour = MCP79410_dec2bcd(time->hour);
    }

    if ((hour & PM) == PM) {
        hour = hour | PM;
    }

    weekday &= 0x38; //Mask 3 upper bits
    weekday |= MCP79410_dec2bcd(time->weekday); //Weekday

    date = MCP79410_dec2bcd(time->date); //Date

    if ((month & LPYR) == LPYR) //Month 
    {
        month = MCP79410_dec2bcd(time->month) | LPYR;
    } else {
        month = MCP79410_dec2bcd(time->month);
    }

    year = MCP79410_dec2bcd(time->year); //Year
    MCP79410_Command(RTCSEC, sec, Write);
    MCP79410_Command(RTCMIN, min, Write);
    MCP79410_Command(RTCHOUR, hour, Write);
    MCP79410_Command(RTCWKDAY, weekday, Write);
    MCP79410_Command(RTCDATE, date, Write);
    MCP79410_Command(RTCMTH, month, Write);
    MCP79410_Command(RTCYEAR, year, Write);
    Nop();
}

void MCP79410_SetHourFormat(Format_t format) {
    MCP79410_DisableOscillator(); //Diable clock
    uint8_t Format_bit = MCP79410_Command(RTCHOUR, 0x00, Read); //Read hour format bit  
    if (format == H24) {
        Format_bit &= ~HOUR_FORMAT; //Set format to H12 (military) 
    } else {
        Format_bit |= HOUR_FORMAT; //Set format to H12
    }
    MCP79410_Command(RTCHOUR, Format_bit, Write);
    MCP79410_EnableOscillator(); //Enable clock
}

void MCP79410_SetPMAM(PMAM_t meridian) {
    MCP79410_DisableOscillator(); //Diable clock
    uint8_t PMAM_bit = MCP79410_Command(RTCHOUR, 0x00, Read); //Read meridian bit 
    if (meridian == AMT) {
        PMAM_bit &= ~PM; //Set AM
    } else {
        PMAM_bit |= PM; //Set PM
    }
    MCP79410_Command(RTCHOUR, PMAM_bit, Write); //Update PM/AM meridian bit
    MCP79410_EnableOscillator(); //Enable clock
}

void MCP79410_EnableAlarm(Alarm_t alarm) {
    uint8_t ctrl_bits = MCP79410_Command(CONTROL, 0x00, Read);
    if (alarm == ZERO) {
        ctrl_bits |= ALM_0;
        MCP79410_Command(CONTROL, ctrl_bits, Write);
    } else {
        ctrl_bits |= ALM_1;
        MCP79410_Command(CONTROL, ctrl_bits, Write);
    }
}

void MCP79410_DisableAlarm(Alarm_t alarm) {
    uint8_t ctrl_bits = MCP79410_Command(CONTROL, 0x00, Read);
    if (alarm == ZERO) {
        ctrl_bits &= ~ALM_0;
        MCP79410_Command(CONTROL, ctrl_bits, Write);
    } else {
        ctrl_bits &= ~ALM_1;
        MCP79410_Command(CONTROL, ctrl_bits, Write);
    }
}

AlarmStatus_t MCP79410_GetAlarmStatus(Alarm_t alarm) {
    AlarmStatus_t status;
    uint8_t temp;
    if (alarm == ZERO) {
        temp = MCP79410_Command(ALM0WKDAY, 0x00, Read); //Read WKDAY register for ALRAM 0  
    } else {
        temp = MCP79410_Command(ALM1WKDAY, 0x00, Read); //Read WKDAY register for ALRAM 1
    }
    return status = (AlarmStatus_t) ((temp & ALMx_IF) == ALMx_IF) ? SET : NOT_SET;
}

void MCP79410_ClearInterruptFlag(Alarm_t alarm) {
    uint8_t temp;
    if (alarm == ZERO) {
        temp = MCP79410_Command(ALM0WKDAY, 0x00, Read); //Read WKDAY register for ALRAM 0   
        temp &= (~ALMx_IF); //Clear 4-th bit 
        MCP79410_Command(ALM0WKDAY, temp, Write); //Enable backup battery mode
    } else {
        temp = MCP79410_Command(ALM1WKDAY, 0x00, Read); //Read WKDAY register for ALRAM 1
        temp &= (~ALMx_IF); //Clear 4-th bit
        MCP79410_Command(ALM1WKDAY, temp, Write); //Enable backup battery mode
    }
}

void MCP79410_SetAlarmTime(RTCC_t *time, Alarm_t alarm) {
    uint8_t sec = MCP79410_dec2bcd(time->sec);
    uint8_t min = MCP79410_dec2bcd(time->min);
    uint8_t hour = MCP79410_dec2bcd(time->hour);
    uint8_t weekday = MCP79410_dec2bcd(time->weekday);
    uint8_t date = MCP79410_dec2bcd(time->date);
    uint8_t month = MCP79410_dec2bcd(time->month);

    if (alarm == ZERO) {
        MCP79410_Command(ALM0SEC, sec | START_32KHZ, Write);
        MCP79410_Command(ALM0MIN, min, Write);
        MCP79410_Command(ALM0HOUR, hour, Write);
        MCP79410_Command(ALM0WKDAY, weekday, Write);
        MCP79410_Command(ALM0DATE, date, Write);
        MCP79410_Command(ALM0MTH, month, Write);

    } else {
        MCP79410_Command(ALM1SEC, sec | START_32KHZ, Write);
        MCP79410_Command(ALM1MIN, min, Write);
        MCP79410_Command(ALM1HOUR, hour, Write);
        MCP79410_Command(ALM1WKDAY, weekday, Write);
        MCP79410_Command(ALM1DATE, date, Write);
        MCP79410_Command(ALM1MTH, month, Write);
    }
}

void MCP79410_SetAlarmMFPPolarity(Polarity_t MFP_pol, Alarm_t alarm) {
    uint8_t Polarity_bit = 0;

    if (alarm == ZERO) {
        Polarity_bit = MCP79410_Command(ALM0WKDAY, 0x00, Read); //Read hour format bit 
    } else {
        Polarity_bit = MCP79410_Command(ALM1WKDAY, 0x00, Read); //Read hour format bit 
    }

    if (MFP_pol == LOWPOL) {
        Polarity_bit &= ~ALMx_POL; //Set MFP LOW
    } else {
        Polarity_bit |= ALMx_POL; //Set MFP HIGH
    }

    if (alarm == ZERO) {
        MCP79410_Command(ALM0WKDAY, Polarity_bit, Write); //Update polarity bit for Alarm 0
    } else {
        MCP79410_Command(ALM1WKDAY, Polarity_bit, Write); //Update polarity bit for Alarm 1
    }
}

void MCP79410_SetAlarmMatch(Match_t match, Alarm_t alarm) {
    RTCCRegisterbits_t AlarmRegister;
    if (alarm == ZERO) {
        AlarmRegister = ALM0WKDAY;
    } else {
        AlarmRegister = ALM1WKDAY;
    }

    uint8_t match_bits = MCP79410_Command(AlarmRegister, 0x00, Read);

    switch (match) {
        case SECONDS_MATCH:
            match_bits &= ~(ALM0MSK2 | ALM0MSK1 | ALM0MSK0);
            MCP79410_Command(AlarmRegister, match_bits, Write); //Minutes match
            break;
        case MINUTES_MATCH:
            match_bits |= ALM0MSK0;
            MCP79410_Command(AlarmRegister, match_bits, Write); //Minutes match
            break;
        case HOURS_MATCH:
            match_bits |= ALM0MSK1;
            MCP79410_Command(AlarmRegister, match_bits, Write); //Hours match
            break;
        case WEEKDAY_MATCH:
            match_bits |= ALM0MSK1 | ALM0MSK0;
            MCP79410_Command(AlarmRegister, match_bits, Write); //Day of week match
            break;
        case DATE_MATCH:
            match_bits |= ALM0MSK2;
            MCP79410_Command(AlarmRegister, match_bits, Write); //Date match
            break;
        case FULL_DATE_MATCH:
            match_bits |= ALM0MSK2 | ALM0MSK1 | ALM0MSK0;
            MCP79410_Command(AlarmRegister, match_bits, Write); //Sec, Minutes Hours, Date match
            break;
        default:
            match_bits |= ALM0MSK0;
            MCP79410_Command(AlarmRegister, match_bits, Write); //Minutes match
            break;
    }
}

void MCP79410_SetMFP_Functionality(MFP_t mode) {
    uint8_t MFP_bits = MCP79410_Command(CONTROL, 0x00, Read);

    switch (mode) {
        case GPO: //For GPO clear SQWEN, ALM0EN, ALM1EN
            MFP_bits &= ~(SQWEN | ALM_0 | ALM_1);
            MCP79410_Command(CONTROL, MFP_bits, Write);
            break;
        case ALARM_INTERRUPT: //For ALARM Interrupts clear SQWEN and set either ALM0EN or ALM1EN
            MFP_bits &= SQWEN;
            MFP_bits |= ALM_0;
            MCP79410_Command(CONTROL, MFP_bits, Write);
            break;
        case SQUARE_WAVE: //For SQUARE WAVE set SQWEN 
            MFP_bits &= ~(ALM_0 | ALM_1);
            MFP_bits |= SQWEN;
            MCP79410_Command(CONTROL, MFP_bits, Write);
            break;
        default: //ALARM Interrupts 
            MFP_bits &= SQWEN;
            MFP_bits |= ALM_0;
            MCP79410_Command(CONTROL, MFP_bits, Write);
            break;
    }
}

void MCP79410_SetMFP_GPOStatus(Polarity_t status) {
    uint8_t gpo_bit = MCP79410_Command(CONTROL, 0x00, Read); //General Purpose Output mode only available when (SQWEN = 0, ALM0EN = 0, and ALM1EN = 0):

    if (status == LOWPOL) {
        gpo_bit = OUT_PIN; //MFP signal level is logic low
        MCP79410_Command(CONTROL, gpo_bit, Write);
    } else { //MFP signal level is logic high
        gpo_bit |= OUT_PIN;
        MCP79410_Command(CONTROL, gpo_bit, Write);
    }
}

uint8_t MCP79410_CheckPowerFailure(void) {
    uint8_t PowerFailure_bit = MCP79410_Command(RTCWKDAY, 0x00, Read); //Read meridian bit   
    uint8_t PowerFail;

    if ((PowerFailure_bit & PWRFAIL) == PWRFAIL) {
        PowerFail = 1;
    } else {
        PowerFail = 0;
    }
    PowerFailure_bit &= ~PWRFAIL; //Clear Power failure bit
    MCP79410_Command(RTCWKDAY, PowerFailure_bit, Write); //Update PM/AM meridian bit

    return PowerFail;
}

uint8_t MCP79410_IsVbatEnabled(void) {
    uint8_t temp;
    temp = MCP79410_Command(RTCWKDAY, 0x00, Read); //The 3rd bit of the RTCC_RTCC day register controls VBATEN   

    if ((temp & VBATEN) == VBATEN) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void MCP79410_EnableVbat(void) {
    uint8_t temp;
    temp = MCP79410_Command(RTCWKDAY, 0x00, Read); //The 3rd bit of the RTCC_RTCC day register controls VBATEN   
    temp = (temp | VBATEN); //Set 3rd bit to enable backup battery mode
    MCP79410_Command(RTCWKDAY, temp, Write); //Enable backup battery mode
}

void MCP79410_DisableVbat(void) {
    uint8_t temp;
    temp = MCP79410_Command(RTCWKDAY, 0x00, Read); //The 3rd bit of the RTCC_RTCC day register controls VBATEN   
    temp = (temp & VBAT_DIS); //Clear 3rd bit to disable backup battery mode
    MCP79410_Command(RTCWKDAY, temp, Write); //Enable backup battery mode    
}

RTCC_t* MCP79410_GetPowerUpTime(void) {
    RTCC_t *powerup_time = (RTCC_t *) malloc(sizeof (RTCC_t));

    powerup_time->min = MCP79410_bcd2dec(MCP79410_Command(PWRUPMIN, 0x00, Read));
    powerup_time->hour = MCP79410_bcd2dec(MCP79410_Command(PWRUPHOUR, 0x00, Read));
    powerup_time->date = MCP79410_bcd2dec(MCP79410_Command(PWRUPDATE, 0x00, Read));
    powerup_time->month = MCP79410_bcd2dec(MCP79410_Command(PWRUPMTH, 0x00, Read));

    return powerup_time;
}

RTCC_t* MCP79410_GetPowerDownTime(void) {
    RTCC_t *powerdown_time = (RTCC_t *) malloc(sizeof (RTCC_t));

    powerdown_time->min = MCP79410_bcd2dec(MCP79410_Command(PWRDNMIN, 0x00, Read));
    powerdown_time->hour = MCP79410_bcd2dec(MCP79410_Command(PWRDNHOUR, 0x00, Read));
    powerdown_time->date = MCP79410_bcd2dec(MCP79410_Command(PWRDNDATE, 0x00, Read));
    powerdown_time->month = MCP79410_bcd2dec(MCP79410_Command(PWRDNMTH, 0x00, Read));

    return powerdown_time;
}

uint8_t MCP79410_dec2bcd(uint8_t num) {
    return ((num / 10 * 16) + (num % 10));
}

uint8_t MCP79410_bcd2dec(uint8_t num) {
    return ((num / 16 * 10) + (num % 16));
}

uint16_t MCP79410_Command(RTCCRegisterbits_t MemoryAddress, uint8_t data, I2CnWR_t nW_R) {
#ifndef USING_SIMULATOR
    __delay_us(7);
#endif
    I2C1_TRANSACTION_REQUEST_BLOCK readTRB[2];
    uint16_t TimeOut = 0, readData = 0;
    uint8_t Command[2], RcvData[1];
    Command[0] = MemoryAddress;
    Command[1] = data;
    if (nW_R == Write) {
        I2C1_MasterWrite(Command, 2, SLAVE_I2C1_MCP79410_REG_ADDRESS, &I2C1MessageStatus);
#ifndef USING_SIMULATOR
        while (I2C1MessageStatus == I2C1_MESSAGE_PENDING) {
            if (TimeOut == SLAVE_I2C1_DEVICE_TIMEOUT) {
                return (0);
            } else TimeOut++;
            if (I2C1MessageStatus == I2C1_MESSAGE_FAIL) {
                return (0);
                break;
            }
        }
#endif
        Nop();
    } else if (nW_R == Read) {
        I2C1_MasterWriteTRBBuild(readTRB, Command, 1, SLAVE_I2C1_MCP79410_REG_ADDRESS);
        I2C1_MasterReadTRBBuild(&readTRB[1], RcvData, 1, SLAVE_I2C1_MCP79410_REG_ADDRESS);
        I2C1_MasterTRBInsert(2, readTRB, &I2C1MessageStatus);
#ifndef USING_SIMULATOR
        while (I2C1MessageStatus == I2C1_MESSAGE_PENDING) {
            if (TimeOut == SLAVE_I2C1_DEVICE_TIMEOUT) {
                return (0);
            } else TimeOut++;
            if (I2C1MessageStatus == I2C1_MESSAGE_FAIL) {
                return (0);
                break;
            }
        }
#endif
        if (I2C1MessageStatus == I2C1_MESSAGE_COMPLETE) {
            readData = RcvData[0];
        }
        Nop();
    }
    return readData;

}