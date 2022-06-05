


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
#define LCD_DATA_DIR            TRISC 
#define LCD_DATA                PORTC 

#define LCD_RS_DIR              TRISDbits.TRISD4     // RS signal for LCD 
#define LCD_RS                  PORTDbits.RD4     // RS signal for LCD 

#define LCD_EN_DIR               TRISDbits.TRISD5     // E signal for LCD 
#define LCD_EN                   PORTDbits.RD5     // E signal for LCD 


#define LCD_RD4     RC4        // D4
#define LCD_RD4_DIR TRISC4

#define LCD_RD5     RC5       // D5
#define LCD_RD5_DIR TRISC5

#define LCD_RD6     RC6       // D6
#define LCD_RD6_DIR TRISC6

#define LCD_RD7     RC7        // D7
#define LCD_RD7_DIR TRISC7


#define LCD_DELAY_MSG     500


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// VARIABLES  //////////////////////////////////
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


// FIXME Delete repeated commands
#define      LCD_FIRST_ROW           128
#define      LCD_SECOND_ROW          192
#define      LCD_THIRD_ROW           144
#define      LCD_FOURTH_ROW          208
#define      LCD_CLEAR               1
#define      LCD_RETURN_HOME         2
#define      LCD_CURSOR_OFF          12
#define      LCD_UNDERLINE_ON        14
#define      LCD_BLINK_CURSOR_ON     15
#define      LCD_MOVE_CURSOR_LEFT    16
#define      LCD_MOVE_CURSOR_RIGHT   20
#define      LCD_TURN_OFF            0
#define      LCD_TURN_ON             8
#define      LCD_SHIFT_LEFT          24
#define      LCD_SHIFT_RIGHT         28

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// PROTOTYPES FUNCTIONS  /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void LCD_Init();

void LCD_out(char * lcd_msg);
void LCD_Message(unsigned char * Msg_Line1, unsigned char * Msg_Line2);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// CONTROL FUNCTIONS  //////////////////////////////
///////////////////////////////////////////////////////////////////////////////


 
/* Write control word to LCD */ 

static unsigned char LCD_nibble(unsigned char data){
    
    char result = 1;
    
    LCD_RD7 = (data & 0x08) >> 3;
    LCD_RD6 = (data & 0x04) >> 2;
    LCD_RD5 = (data & 0x02) >> 1;
    LCD_RD4 = (data & 0x01) >> 0;
    LCD_EN    = HIGH; 
    NOP();
    LCD_EN    = LOW; 
    _delay(1000);
    
    result = 0;
    
    return result;
    
}


static unsigned char LCD_Cmd(unsigned char lcd_command){  
    
    LCD_RD7_DIR = 1;
    while(LCD_RD7);
    LCD_RD7_DIR = 0;     
    
    LCD_RS   = LOW;         
    LCD_EN    = LOW;         
    LCD_nibble((char) ((lcd_command & 0xF0) >> 4));
    LCD_nibble(lcd_command);
   
} 


 
/* Write text data to LCD */ 
static unsigned char LCD_sendData(unsigned char lcd_data){    
    
    LCD_RD7_DIR = 1;
    while(LCD_RD7);
    LCD_RD7_DIR = 0;
        
    LCD_EN    = LOW;   
    LCD_RS   = HIGH;   
    LCD_nibble((char) ((lcd_data & 0xF0) >> 4));
    LCD_nibble(lcd_data);
    
    LCD_RD7_DIR = 1;
    while(LCD_RD7);
    LCD_RD7_DIR = 0;
    
}

// LCD MASTER FUNCTION
void LCD_out(char * lcd_msg){
    for (int j=0; lcd_msg[j]!=0; j++)     
        LCD_sendData(lcd_msg[j]); 
}

/* LCD display initialization */ 
void LCD_Init(){    
    
    LCD_RS_DIR = 0;
    LCD_EN_DIR = 0;
    
    LCD_RD4_DIR = 0;
    LCD_RD5_DIR = 0;
    LCD_RD6_DIR = 0;
    LCD_RD7_DIR = 0;
    
    _delay(15 * 1000);

    
    LCD_Cmd(RETURN_HOME);            // Return cursor to home position 
    LCD_Cmd(LCD_2LINES_5X8);         // Function Set - 8-bit, 2 lines, 5X7 
    LCD_Cmd(CLEAR_LCD);              // Clear display   
    LCD_Cmd(DISPLAY_ON_CURSOR_OFF);  // Display on, cursor off 
    LCD_Cmd(INCREMENT_CURSOR);       // Entry mode - inc addr, no shift 
} 

void LCD_Message(char * LINE1_MSG, char * LINE2_MSG){
    
    LCD_Cmd(CLEAR_LCD);              // Clear display  
    
    LCD_Cmd(HOME_LINE1);
    for (int j = 0; LINE1_MSG[j] != 0; j++)     
        LCD_sendData(LINE1_MSG[j]); 
//    Lcd_Out(1, 8 - (strlen(buffer) / 2), buffer);
    
    LCD_Cmd(HOME_LINE2);
    for (int j = 0; LINE2_MSG[j] != 0; j++)     
        LCD_sendData(LINE2_MSG[j]); 
//    Lcd_Out(1, 8 - (strlen(buffer) / 2), buffer);
        
    __delay_ms(LCD_DELAY_MSG);
    
}

