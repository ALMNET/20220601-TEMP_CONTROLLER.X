////////////////////////////////////////////////////////////////////////////////
////////////////////////// I/O PINOUT AND DEFINITIONS //////////////////////////
////////////////////////////////////////////////////////////////////////////////

// LCD
#define LCD_DATA_DIR            TRISC 
#define LCD_DATA                PORTC 

#define LCD_RS_DIR              TRISDbits.TRISD4     // RS signal for LCD 
#define LCD_RS                  PORTDbits.RD4     // RS signal for LCD 

#define LCD_EN_DIR               TRISDbits.TRISD5     // E signal for LCD 
#define LCD_EN                   PORTDbits.RD5     // E signal for LCD 

#define LCD_DELAY_MSG     500


//#define LCD_RS      RD2       // RS
//#define TRISRS      TRISD2
//
//#define LCD_EN      RD3      // EN
//#define TRISEN      TRISD3
//
//
#define LCD_RD4     RC4        // D4
#define LCD_RD4_DIR TRISC4

#define LCD_RD5     RC5       // D5
#define LCD_RD5_DIR TRISC5

#define LCD_RD6     RC6       // D6
#define LCD_RD6_DIR TRISC6

#define LCD_RD7     RC7        // D7
#define LCD_RD7_DIR TRISC7

#define HIGH    1
#define LOW     0

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// VARIABLES  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

unsigned char i; 

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
void Lcd_Cmd(unsigned char lcd_command);
void LCD_sendData(unsigned char lcd_data);
void LCD_MESSAGE(char * GENERAL_MESSAGE);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// CONTROL FUNCTIONS  //////////////////////////////
///////////////////////////////////////////////////////////////////////////////


 
/* Write control word to LCD */ 

static void LCD_nibble(unsigned char data){
    
//    char result = 1;
    
    LCD_RD7 = (data & 0x08) >> 3;
    LCD_RD6 = (data & 0x04) >> 2;
    LCD_RD5 = (data & 0x02) >> 1;
    LCD_RD4 = (data & 0x01) >> 0;
    LCD_EN    = HIGH; 
    NOP();
    LCD_EN    = LOW; 
    _delay(1000);
    
//    result = 0;
//    
//    return result;
    
}


void Lcd_Cmd(unsigned char lcd_command){  
    
    LCD_RD7_DIR = 1;
    while(LCD_RD7);
    LCD_RD7_DIR = 0;     
    
    LCD_RS   = LOW;         
    LCD_EN    = LOW;         
    LCD_nibble((char) ((lcd_command & 0xF0) >> 4));
    LCD_nibble(lcd_command);
   
} 


 
/* Write text data to LCD */ 
void LCD_sendData(unsigned char lcd_data){    
    
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

void LCD_MESSAGE(char * GENERAL_MESSAGE){
    for (int j=0; GENERAL_MESSAGE[j]!=0; j++)     
        LCD_sendData(GENERAL_MESSAGE[j]); 
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

    
    Lcd_Cmd(RETURN_HOME);            // Return cursor to home position 
    Lcd_Cmd(LCD_2LINES_5X8);         // Function Set - 8-bit, 2 lines, 5X7 
    Lcd_Cmd(CLEAR_LCD);              // Clear display   
    Lcd_Cmd(DISPLAY_ON_CURSOR_OFF);  // Display on, cursor off 
    Lcd_Cmd(INCREMENT_CURSOR);       // Entry mode - inc addr, no shift 
} 

//    for(long y = 0x00; y < 0xF0; y=y+0x10){
//        LCD_nibble((y & 0xF0) >> 4);
//        for(long x = 0; x < 1000; x++) _delay(1000);
//    }
    
//    unsigned long x;
//    
//    LCD_nibble((0x80 & 0xF0) >> 4);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    LCD_nibble((0x40 & 0xF0) >> 4);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    LCD_nibble((0x20 & 0xF0) >> 4);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    LCD_nibble((0x10 & 0xF0) >> 4);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    
//    LCD_nibble(0x08);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    LCD_nibble(0x04);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    LCD_nibble(0x02);
//    for(x = 0; x < 2000; x++) _delay(1000);
//    LCD_nibble(0x01);
//    for(x = 0; x < 2000; x++) _delay(1000);
    
//    Lcd_Cmd(0x53);         // Function Set - 8-bit, 2 lines, 5X7 