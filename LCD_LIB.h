/*!
 * @file LCD_LIB.h
 *
 * 
 */

#ifndef _LCD_LIB_
#define _LCD_LIB_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <xc.h>

////////////////////////////////////////////////////////////////////////////////
////////////////////////// I/O PINOUT AND DEFINITIONS //////////////////////////
////////////////////////////////////////////////////////////////////////////////


#define HIGH    1
#define LOW     0

// LCD

#define LCD_RS_DIR  TRISDbits.TRISD4     // RS signal for LCD 
#define LCD_RS      PORTDbits.RD4     // RS signal for LCD 

#define LCD_EN_DIR  TRISDbits.TRISD5     // E signal for LCD 
#define LCD_EN      PORTDbits.RD5     // E signal for LCD 


#define LCD_RD4     RC4        // D4
#define LCD_RD4_DIR TRISC4

#define LCD_RD5     RC5       // D5
#define LCD_RD5_DIR TRISC5

#define LCD_RD6     RC6       // D6
#define LCD_RD6_DIR TRISC6

#define LCD_RD7     RC7        // D7
#define LCD_RD7_DIR TRISC7


#define LCD_DELAY_MSG     1000


///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// LCD COMMANDS  /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define CLEAR_LCD               0x01
#define RETURN_HOME             0x02
#define DECREMENT_CURSOR        0x04
#define INCREMENT_CURSOR        0x06
#define SHIFT_RIGHT             0x05
#define SHIFT_LEFT              0x07
#define DISPLAY_OFF_CURSOR_OFF  0x08
#define DISPLAY_OFF_CURSOR_ON   0x0A
#define DISPLAY_ON_CURSOR_OFF   0x0C
#define DISPLAY_ON_CURSOR_ON    0x0E
#define SHIFT_CURSOR_TO_LEFT    0x10
#define SHIFT_CURSOR_TO_RIGHT   0x14
#define SHIFT_DISPLAY_LEFT      0x18
#define SHIFT_DISPLAY_RIGHT     0x1C
#define HOME_LINE1              0x80
#define HOME_LINE2              0xC0
#define LCD_2LINES_5X7          0x38
#define LCD_2LINES_5X8          0x28

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// PROTOTYPES FUNCTIONS  /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static unsigned char LCD_nibble(unsigned char data);
unsigned char LCD_Cmd(unsigned char lcd_command);
unsigned char LCD_sendData(unsigned char lcd_data);
unsigned char LCD_out(char pos_y, char pos_x, char * lcd_msg);
void LCD_out_CP(char * lcd_msg);

void LCD_Message(char LINE1_MSG[], char LINE2_MSG[]);
void LCD_Init();


#endif