/*
 * File:   MCP79410.c
 * Author: user
 *
 * Created on 2020年12月21日, 上午 9:01
 */


#include "Common.h"

extern struct _clock CLOCK;
I2C1_MESSAGE_STATUS i2c1_msg_status;
RTCC_Struct CurrentTime;

void MCP79410_Initialize(void) {
    MCP79410_SetHourFormat(H24); //Set hour format to military time standard
    MCP79410_EnableVbat(); //Enable battery backup
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    RTCC_Struct curr_time = {tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_wday, tm.tm_mday, tm.tm_mon + 1, ((tm.tm_year + 1900) - 2000)};
#ifdef __DEBUG_RTCC__
    printf("MCP79410_Initialize DEBUG now: %d-%d-%d %d:%d:%d\n", tm.tm_year, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
#endif
    curr_time.year = 20;
    curr_time.month = 12;
    curr_time.date = 28;
    curr_time.weekday = 1;
    curr_time.hour = 8;
    curr_time.min = 20;
    curr_time.sec = 0;
    MCP79410_SetTime(&curr_time);
    MCP79410_EnableOscillator(); //Start clock by enabling oscillator

}

void MCP79410_EnableOscillator(void) {
    uint16_t ST_bit = MCP79410_Command(RTCWKDAY, 0x00, Read); //Read day + OSCON bit
    ST_bit = ST_bit | START_32KHZ;
    MCP79410_Command(RTCSEC, ST_bit, Write); //START bit is located in the Sec register
}

void MCP79410_DisableOscillator(void) {
    uint16_t ST_bit = MCP79410_Command(RTCWKDAY, 0x00, Read); //Read day + OSCON bit
    ST_bit = ST_bit & ~START_32KHZ;
    MCP79410_Command(RTCSEC, ST_bit, Write); //START bit is located in the Sec regist
}

unsigned char MCP79410_IsRunning(void) {
    unsigned char mask = MCP79410_Command(RTCWKDAY, 0x00, Read);

    if ((mask & OSCRUN) == OSCRUN) //If oscillator = already running, do nothing.
    {
        return TRUE;
    } else {
        return FALSE;
    }
}

void MCP79410_GetTime(void) {
    CurrentTime.sec = MCP79410_bcd2dec(MCP79410_Command(RTCSEC, 0x00, Read) & (~START_32KHZ));
    CurrentTime.min = MCP79410_bcd2dec(MCP79410_Command(RTCMIN, 0x00, Read));

    unsigned char hour_t = MCP79410_Command(RTCHOUR, 0x00, Read);

    hour_t = ((hour_t & HOUR_12) == HOUR_12) ? (hour_t & 0x1F) : (hour_t & 0x3F); //hour is in 24 hour format

    CurrentTime.hour = MCP79410_bcd2dec(hour_t);
    CurrentTime.weekday = MCP79410_bcd2dec(MCP79410_Command(RTCWKDAY, 0x00, Read) & ~(OSCRUN | PWRFAIL | VBATEN));
    CurrentTime.date = MCP79410_bcd2dec(MCP79410_Command(RTCDATE, 0x00, Read));
    CurrentTime.month = MCP79410_bcd2dec(MCP79410_Command(RTCMTH, 0x00, Read) & ~(LPYR));
    CurrentTime.year = MCP79410_bcd2dec(MCP79410_Command(RTCYEAR, 0x00, Read));

    CLOCK.Year = CurrentTime.year;
    CLOCK.Month = CurrentTime.month;
    CLOCK.Date = CurrentTime.date;
    CLOCK.Weekday = CurrentTime.weekday;
    CLOCK.Hour = CurrentTime.hour;
    CLOCK.Minute = CurrentTime.min;
    CLOCK.Second = CurrentTime.sec;
    Nop();

}

void MCP79410_SetTime(RTCC_Struct *time) {
    unsigned char sec = MCP79410_Command(RTCSEC, 0x00, Read); //Seconds
    unsigned char min = 0; //Minutes
    unsigned char hour = MCP79410_Command(RTCHOUR, 0x00, Read); //Hours
    unsigned char weekday = MCP79410_Command(RTCWKDAY, 0x00, Read); //Weekday
    unsigned char date = 0; //Date
    unsigned char month = MCP79410_Command(RTCMTH, 0x00, Read); //Month
    unsigned char year = 0;
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
    unsigned char Format_bit = MCP79410_Command(RTCHOUR, 0x00, Read); //Read hour format bit  
    if (format == H24) {
        Format_bit &= ~HOUR_FORMAT; //Set format to H12 (military) 
    } else {
        Format_bit |= HOUR_FORMAT; //Set format to H12
    }
    MCP79410_Command(RTCHOUR, Format_bit, Write); //START bit is located in the Sec register
    MCP79410_EnableOscillator(); //Enable clock
}

void MCP79410_SetPMAM(PMAM_t meridian) {
    MCP79410_DisableOscillator(); //Diable clock
    unsigned char PMAM_bit = MCP79410_Command(RTCHOUR, 0x00, Read); //Read meridian bit 
    if (meridian == AMT) {
        PMAM_bit &= ~PM; //Set AM
    } else {
        PMAM_bit |= PM; //Set PM
    }
    MCP79410_Command(RTCHOUR, PMAM_bit, Write); //Update PM/AM meridian bit
    MCP79410_EnableOscillator(); //Enable clock
}

void MCP79410_EnableAlarm(Alarm_t alarm) {
    unsigned char ctrl_bits = MCP79410_Command(CONTROL, 0x00, Read);
    if (alarm == ZERO) {
        ctrl_bits |= ALM_0;
        MCP79410_Command(CONTROL, ctrl_bits, Write);
    } else {
        ctrl_bits |= ALM_1;
        MCP79410_Command(CONTROL, ctrl_bits, Write);
    }
}

void MCP79410_DisableAlarm(Alarm_t alarm) {
    unsigned char ctrl_bits = MCP79410_Command(CONTROL, 0x00, Read);
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
    unsigned char temp;
    if (alarm == ZERO) {
        temp = MCP79410_Command(ALM0WKDAY, 0x00, Read); //Read WKDAY register for ALRAM 0  
    } else {
        temp = MCP79410_Command(ALM1WKDAY, 0x00, Read); //Read WKDAY register for ALRAM 1
    }
    return status = (AlarmStatus_t) ((temp & ALMx_IF) == ALMx_IF) ? SET : NOT_SET;
}

void MCP79410_ClearInterruptFlag(Alarm_t alarm) {
    unsigned char temp;
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

void MCP79410_SetAlarmTime(RTCC_Struct *time, Alarm_t alarm) {
    unsigned char sec = MCP79410_dec2bcd(time->sec);
    unsigned char min = MCP79410_dec2bcd(time->min);
    unsigned char hour = MCP79410_dec2bcd(time->hour);
    unsigned char weekday = MCP79410_dec2bcd(time->weekday);
    unsigned char date = MCP79410_dec2bcd(time->date);
    unsigned char month = MCP79410_dec2bcd(time->month);

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
    unsigned char Polarity_bit = 0;

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
    rtcc_registerbits AlarmRegister;
    if (alarm == ZERO) {
        AlarmRegister = ALM0WKDAY;
    } else {
        AlarmRegister = ALM1WKDAY;
    }

    unsigned char match_bits = MCP79410_Command(AlarmRegister, 0x00, Read);

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
    unsigned char MFP_bits = MCP79410_Command(CONTROL, 0x00, Read);

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
    unsigned char gpo_bit = MCP79410_Command(CONTROL, 0x00, Read); //General Purpose Output mode only available when (SQWEN = 0, ALM0EN = 0, and ALM1EN = 0):

    if (status == LOWPOL) {
        gpo_bit = OUT_PIN; //MFP signal level is logic low
        MCP79410_Command(CONTROL, gpo_bit, Write);
    } else { //MFP signal level is logic high
        gpo_bit |= OUT_PIN;
        MCP79410_Command(CONTROL, gpo_bit, Write);
    }
}

unsigned char MCP79410_CheckPowerFailure(void) {
    unsigned char PowerFailure_bit = MCP79410_Command(RTCWKDAY, 0x00, Read); //Read meridian bit   
    unsigned char PowerFail;

    if ((PowerFailure_bit & PWRFAIL) == PWRFAIL) {
        PowerFail = 1;
    } else {
        PowerFail = 0;
    }
    PowerFailure_bit &= ~PWRFAIL; //Clear Power failure bit
    MCP79410_Command(RTCWKDAY, PowerFailure_bit, Write); //Update PM/AM meridian bit

    return PowerFail;
}

unsigned char MCP79410_IsVbatEnabled(void) {
    unsigned char temp;
    temp = MCP79410_Command(RTCWKDAY, 0x00, Read); //The 3rd bit of the RTCC_RTCC day register controls VBATEN   

    if ((temp & VBATEN) == VBATEN) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void MCP79410_EnableVbat(void) {
    unsigned char temp;
    temp = MCP79410_Command(RTCWKDAY, 0x00, Read); //The 3rd bit of the RTCC_RTCC day register controls VBATEN   
    temp = (temp | VBATEN); //Set 3rd bit to enable backup battery mode
    MCP79410_Command(RTCWKDAY, temp, Write); //Enable backup battery mode
}

void MCP79410_DisableVbat(void) {
    unsigned char temp;
    temp = MCP79410_Command(RTCWKDAY, 0x00, Read); //The 3rd bit of the RTCC_RTCC day register controls VBATEN   
    temp = (temp & VBAT_DIS); //Clear 3rd bit to disable backup battery mode
    MCP79410_Command(RTCWKDAY, temp, Write); //Enable backup battery mode    

}

RTCC_Struct* MCP79410_GetPowerUpTime(void) {
    RTCC_Struct *powerup_time = (RTCC_Struct *) malloc(sizeof (RTCC_Struct));

    powerup_time->min = MCP79410_bcd2dec(MCP79410_Command(PWRUPMIN, 0x00, Read));
    powerup_time->hour = MCP79410_bcd2dec(MCP79410_Command(PWRUPHOUR, 0x00, Read));
    powerup_time->date = MCP79410_bcd2dec(MCP79410_Command(PWRUPDATE, 0x00, Read));
    powerup_time->month = MCP79410_bcd2dec(MCP79410_Command(PWRUPMTH, 0x00, Read));

    return powerup_time;
}

RTCC_Struct* MCP79410_GetPowerDownTime(void) {
    RTCC_Struct *powerdown_time = (RTCC_Struct *) malloc(sizeof (RTCC_Struct));

    powerdown_time->min = MCP79410_bcd2dec(MCP79410_Command(PWRDNMIN, 0x00, Read));
    powerdown_time->hour = MCP79410_bcd2dec(MCP79410_Command(PWRDNHOUR, 0x00, Read));
    powerdown_time->date = MCP79410_bcd2dec(MCP79410_Command(PWRDNDATE, 0x00, Read));
    powerdown_time->month = MCP79410_bcd2dec(MCP79410_Command(PWRDNMTH, 0x00, Read));

    return powerdown_time;
}

unsigned char MCP79410_dec2bcd(unsigned char num) {
    return ((num / 10 * 16) + (num % 10));
}

unsigned char MCP79410_bcd2dec(unsigned char num) {
    return ((num / 16 * 10) + (num % 16));
}

uint16_t MCP79410_Command(rtcc_registerbits MemoryAddress, uint8_t data, I2CnW_R nW_R) {
#ifndef USING_SIMULATOR
    __delay_us(7);
#endif

    I2C1_TRANSACTION_REQUEST_BLOCK readTRB[2];
    unsigned int TimeOut = 0, readData = 0;
    unsigned char Command[2], RecData[1];
    Command[0] = MemoryAddress;
    Command[1] = data;
    if (nW_R == Write) {
        I2C1_MasterWrite(Command, 2, SLAVE_I2C1_MCP79410_REG_ADDRESS, &i2c1_msg_status);
#ifndef USING_SIMULATOR
        while (i2c1_msg_status == I2C1_MESSAGE_PENDING) {
            if (TimeOut == SLAVE_I2C1_MCP79410_DEVICE_TIMEOUT) {
                return (0);
            } else TimeOut++;
            if (i2c1_msg_status == I2C1_MESSAGE_FAIL) {
                return (0);
                break;
            }
        }
#endif
        Nop();
    } else if (nW_R == Read) {
        // Build TRB for sending address
        I2C1_MasterWriteTRBBuild(readTRB, Command, 1, SLAVE_I2C1_MCP79410_REG_ADDRESS);
        // Build TRB for receiving data
        I2C1_MasterReadTRBBuild(&readTRB[1], RecData, 1, SLAVE_I2C1_MCP79410_REG_ADDRESS);
        I2C1_MasterTRBInsert(2, readTRB, &i2c1_msg_status);
#ifndef USING_SIMULATOR
        while (i2c1_msg_status == I2C1_MESSAGE_PENDING) {
            if (TimeOut == SLAVE_I2C1_MCP79410_DEVICE_TIMEOUT) {
                return (0);
            } else TimeOut++;
            if (i2c1_msg_status == I2C1_MESSAGE_FAIL) {
                return (0);
                break;
            }
        }
#endif
        if (i2c1_msg_status == I2C1_MESSAGE_COMPLETE) {
            readData = RecData[0];
        }
        Nop();
    }
    return readData;

}