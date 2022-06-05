// This is a guard condition so that contents of this file are not included
// more than once.  
//#ifndef XC_HEADER_TEMPPORTE_H
//#define	XC_HEADER_TEMPPORTE_H

#include <xc.h> // include processor files - each processor file is guarded. 

#ifndef _XTAL_FREQ
#define _XTAL_FREQ   4000000
#endif
////////////////////////////////////////////////////////////////////////////////
////////////////////////// I/O PINOUT AND DEFINITIONS //////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define LCD_RS      RD2       // RS
#define TRISRS      TRISD2

#define LCD_EN      RD3      // EN
#define TRISEN      TRISD3


#define LCD_RD4     RD4        // D4
#define TRISRD4     TRISD4

#define LCD_RD5     RD5       // D5
#define TRISRD5     TRISD5

#define LCD_RD6     RD6       // D6
#define TRISRD6     TRISD6

#define LCD_RD7     RD7        // D7
#define TRISRD7     TRISD7

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

#define LCD_DELAY_MSG     500

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PROTOTYPES //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void Lcd_Init(void);
void Lcd_Out(unsigned char y, unsigned char x, const char *buffer);
void Lcd_Out2(unsigned char y, unsigned char x, char *buffer);
void Lcd_Chr_CP(char data);
void Lcd_Cmd(unsigned char data);
void Lcd_Out_CP(char *buffer);
void Lcd_Position(unsigned char y, unsigned char x);
//extern void __delay_us(long delay_value);

void Lcd_Init(void){
    unsigned char data;
    TRISRD7 = 0;
    TRISRD6 = 0;
    TRISRD5 = 0;
    TRISRD4 = 0;
    TRISEN = 0;
    TRISRS = 0;
    LCD_RD7 = 0;
    LCD_RD6 = 0;
    LCD_RD5 = 0;
    LCD_RD4 = 0;
    LCD_EN = 0;
    LCD_RS = 0;
    __delay_us(5500);
    __delay_us(5500);
    __delay_us(5500);
    __delay_us(5500);
    __delay_us(5500);
    __delay_us(5500);
    for(data = 1; data < 4; data ++)
    {
        LCD_RD7 = 0;    LCD_RD6 = 0;    LCD_RD5 = 1;    LCD_RD4 = 1;    LCD_EN = 0;
        LCD_RS = 0;    LCD_RD7 = 0;    LCD_RD6 = 0;    LCD_RD5 = 1;    LCD_RD4 = 1;
        LCD_EN = 1;    LCD_RS = 0;
        __delay_us(5);
        LCD_RD7 = 0;    LCD_RD6 = 0;    LCD_RD5 = 1;    LCD_RD4 = 1;    LCD_EN = 0;
        LCD_RS = 0;
        __delay_us(5500);
    }
    LCD_RD7 = 0; LCD_RD6 = 0; LCD_RD5 = 1; LCD_RD4 = 0; LCD_EN = 0; LCD_RS = 0;
    LCD_RD7 = 0; LCD_RD6 = 0; LCD_RD5 = 1; LCD_RD4 = 0; LCD_EN = 1; LCD_RS = 0;
    __delay_us(5);
    LCD_RD7 = 0; LCD_RD6 = 0; LCD_RD5 = 1; LCD_RD4 = 0; LCD_EN = 0; LCD_RS = 0;
    __delay_us(5500);
    data = 40; Lcd_Cmd(data);
    data = 16; Lcd_Cmd(data);
    data = 1;  Lcd_Cmd(data);
    data = 15; Lcd_Cmd(data);
}

void Lcd_Out(unsigned char y, unsigned char x, const char *buffer)
{
    unsigned char data;
    switch (y)
    {   
        case 1: data = LCD_FIRST_ROW + x; break;
        case 2: data = LCD_SECOND_ROW + x; break;
        case 3: data = LCD_THIRD_ROW + x; break;
        case 4: data = LCD_FOURTH_ROW + x; break;
        default: break;
    }
    Lcd_Cmd(data);
    while(*buffer)              // Write data to LCD up to null
    {                
        Lcd_Chr_CP(*buffer);
        buffer++;               // Increment buffer
    }
    return;
}


void Lcd_Out2(unsigned char y, unsigned char x, char *buffer)
{
    unsigned char data;
    switch (y)
    {
        case 1: data = LCD_FIRST_ROW + x; break;
        case 2: data = LCD_SECOND_ROW + x; break;
        case 3: data = LCD_THIRD_ROW + x; break;
        case 4: data = LCD_FOURTH_ROW + x; break;
        default: break;
    }
    Lcd_Cmd(data);
    while(*buffer)              // Write data to LCD up to null
    {                
        Lcd_Chr_CP(*buffer);
        buffer++;               // Increment buffer
    }
    return;
}


void Lcd_Chr_CP(char data){
    LCD_EN = 0; 
    LCD_RS = 1;
    LCD_RD7 = (data & 0b10000000)>>7; 
    LCD_RD6 = (data & 0b01000000)>>6;
    LCD_RD5 = (data & 0b00100000)>>5; 
    LCD_RD4 = (data & 0b00010000)>>4;
    __delay_us (10);
    LCD_EN = 1; 
    __delay_us(5); 
    LCD_EN = 0;
    LCD_RD7 = (data & 0b00001000)>>3; 
    LCD_RD6 = (data & 0b00000100)>>2;
    LCD_RD5 = (data & 0b00000010)>>1; 
    LCD_RD4 = (data & 0b00000001);
    __delay_us (10);
    LCD_EN = 1; 
    __delay_us(5); 
    LCD_EN = 0;
    __delay_us(5); 
    __delay_us(5500);
}


void Lcd_Cmd(unsigned char data){
    LCD_EN = 0; LCD_RS = 0;
    LCD_RD7 = (data & 0b10000000)>>7; 
    LCD_RD6 = (data & 0b01000000)>>6;
    LCD_RD5 = (data & 0b00100000)>>5; 
    LCD_RD4 = (data & 0b00010000)>>4;
    __delay_us (10);
    LCD_EN = 1; 
    __delay_us(5); 
    LCD_EN = 0;
    LCD_RD7 = (data & 0b00001000)>>3; 
    LCD_RD6 = (data & 0b00000100)>>2;
    LCD_RD5 = (data & 0b00000010)>>1; 
    LCD_RD4 = (data & 0b00000001);
    __delay_us (10);
    LCD_EN = 1; 
    __delay_us(5); 
    LCD_EN = 0;
    __delay_us(5500);//Delay_5us();
}

void Lcd_Out_CP(char *buffer){
    while(*buffer)              // Write data to LCD up to null
    {                
        Lcd_Chr_CP(*buffer);
        buffer++;               // Increment buffer
    }
    return;    
}

void Lcd_Position(unsigned char y, unsigned char x){
    unsigned char data;
    switch (y)
    {
        case 1: data = LCD_FIRST_ROW + x; break;
        case 2: data = LCD_SECOND_ROW + x; break;
        case 3: data = LCD_THIRD_ROW + x; break;
        case 4: data = LCD_FOURTH_ROW + x; break;
        default: break;
    }
    Lcd_Cmd(data);
}


//#endif	/* XC_HEADER_TEMPPORTE_H */

