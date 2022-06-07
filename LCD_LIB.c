/*!
 * @file LCD_LIB.c
 *
 * 
 */


#include "LCD_LIB.h"
#include <xc.h>


/**
 * @brief       Sends a nibble (4 bit data) to the LCD data bus
 *              this function is not useful for user, its is
 *              just required for LCD_Cmd and LCD_sendData functions
 * 
 * @param       data:
 *              data to be sent to the LCD
 * 
 * @return      0 if the nibble is sucessfully sent
 *              1 if not
 * 
 */

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

/**
 * @brief       Send a command to the LCD bus
 *              
 *   
 * @param[in]   lcd_command: 
 *              The command which will be sent to the LCD
 *              The command is descripted in the LCD header file
 * 
 * @return      0 if the command is sucessfully sent
 *              1 if not
 */

unsigned char LCD_Cmd(unsigned char lcd_command){  
    
    char result = 1;
    
    LCD_RD7_DIR = 1;
    while(LCD_RD7);
    LCD_RD7_DIR = 0;     
    
    LCD_RS   = LOW;         
    LCD_EN    = LOW;         
    LCD_nibble((char) ((lcd_command & 0xF0) >> 4));
    LCD_nibble(lcd_command);
    
    result = 0;
    
    return result;
   
} 
 
/**
 * @brief       Send a command to the LCD bus
 *              
 *   
 * @param[in]   lcd_command: 
 *              The command which will be sent to the LCD
 *              The command is descripted in the LCD header file
 * 
 * @return      0 if the command is sucessfully sent
 *              1 if not
 */

unsigned char LCD_sendData(unsigned char lcd_data){    
    
    char result = 1;
    
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
    
    result = 0;
    
    return result;
    
}

/**
 * @brief       Send a text to the LCD using a specific cursor position
 *              
 *   
 * @param[in]   pos_y: 
 *              y position coord. must be 1 or 2, otherwise the 
 *              function will return an error (1)
 * @param[in]   pos_x: 
 *              x position coord. must be between 1 to 16, otherwise the 
 *              function will return an error (1)
 * @param[in]   lcd_msg: 
 *              Pointer / Array to the message to send. If the message
 *              is empty, the function will return 2
 * 
 * @return      0 if the command is sucessfully sent
 *              1 if the position is wrong
 *              2 if the message is empty
 */

unsigned char LCD_out(char pos_y, char pos_x, char * lcd_msg){
    
    unsigned char result;
    unsigned char cursor;
    
    if(*lcd_msg){
        
        if((pos_x > 0 && pos_x < 17) && (pos_y > 0 && pos_y < 3))
        {
            
            switch(pos_y){
                case 1:
                    LCD_Cmd(HOME_LINE1);
                    break;
                    
                case 2:
                    LCD_Cmd(HOME_LINE2);
                    break;
            }
            
            for(cursor = 1; cursor <= pos_x; cursor++)
                LCD_sendData(' ');
            
            LCD_out_CP(lcd_msg);
            
            // Use this lines in a stack overflow case
//            for (cursor = 0; lcd_msg[cursor] != 0; cursor++)     
//            LCD_sendData(lcd_msg[cursor]); 
            
        }
        
        else
            result = 1;
    }
    
    else
        result = 2;
    
    return result;
}


/**
 * @brief       Send a text to the LCD using the current cursor position
 *              
 *   
 * @param[in]   lcd_msg: 
 *              Pointer / Array to the message to send
 * 
 * @return      none (void function)
 */

void LCD_out_CP(char * lcd_msg){
    for (int j=0; lcd_msg[j]!=0; j++)     
        LCD_sendData(lcd_msg[j]); 
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
      
    
    // LCD_DELAY_MSG screens delay
    for(int counter = 0; counter < LCD_DELAY_MSG; counter++)    _delay(1000);
    
}

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
