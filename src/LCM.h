/* 
 * File:   LCM.h
 * Author: user
 *
 * Created on 2020年4月30日, 下午 12:39
 */

#ifndef LCM_H
#define	LCM_H

#include "Common.h"

#ifdef	__cplusplus
extern "C" {
#endif

    void LCMInitialize(void);
    void LCM_SetCursor(uint8_t Y, uint8_t X);
    void LCM_PutASCII(uint8_t Ascii);
    void LCM_PutHex(uint8_t Hex);
    void LCM_PutROMString(const uint8_t *String);
    void LCM_PutRAMString(uint8_t *String);
    void LCM_PutNumber(uint16_t Number, uint8_t Digit);

    // LCD Module Commands
#define Disp_2Line_4Bit_5x8Dots		0x28
#define Disp_2Line_8Bit_5x8Dots		0x38

#define Disp_Off					0x08		// Display off
#define Disp_On						0x0c		// Display on
#define Disp_On_Cursor				0x0e		// Display on, Cursor on
#define Disp_On_Cursor_Blink		0x0f		// Display on, Cursor on, Blink cursor

#define Disp_Clear					0x01		// Clear the Display

#define Disp_Entry_Inc				0x06		//
#define Disp_Entry_Dec				0x04		//


#ifdef	__cplusplus
}
#endif

#endif	/* LCM_H */

