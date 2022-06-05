////////////////////////////////////////////////////////////////////////////////

/*
 * File:   Main.c
 * Author: ALMNET
 *
 * Created on may 8, 2022, 16:14
 */

////////////////////////////////////////////////////////////////////////////////
////////////////////////// HEADERS AND OTHER LIBRARIES /////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "Fuses.h"

#define _XTAL_FREQ 4000000

//#include "LCD.h"
#include "LCD_LIB.h"



////////////////////////////////////////////////////////////////////////////////
//////////////////////////// TEMPERATURE DEFINITIONS ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Temperature Setpoints in celsius
#define	LO_TEMP     20
#define	MED_TEMP    25
#define	HI_TEMP     30

////////////////////////////////////////////////////////////////////////////////
////////////////////////// I/O PINOUT AND DEFINITIONS //////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define INPUT_PORT  255
#define OUTPUT_PORT 0

#define INPUT       1
#define OUTPUT      0

#define ON          1
#define OFF         0

#define DISP_TIME   1

//////////////////////////////////// INPUTS ////////////////////////////////////

// PORTA 
#define TEMP_CHANNEL_DIR    TRISA0
#define TEMP_CHANNEL        RA0

/////////////////////////////////// OUTPUTS ////////////////////////////////////
// PORTB
#define DISP1_PORT_DIR      TRISB
#define DISP1_PORT          PORTB

// PORTC
#define DISP2_PORT_DIR      TRISC
#define DISP2_PORT          PORTC

// PORTE
#define FAN_1_DIR           TRISE0
#define FAN_1               RE0

#define FAN_2_DIR           TRISE1
#define FAN_2               RE1

#define FAN_3_DIR           TRISE2
#define FAN_3               RE2
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// VARIABLES  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Temperature value obtained from ADC Read
unsigned int temperature;

char buffer[64];

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// PROTOTYPES FUNCTIONS  /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void delay_ms(unsigned long delay_value);

void ADC_Init(void);
unsigned int ADC_Read(void);
//
//void Fan_config_1();
//void Fan_config_2();
//void Fan_config_3();

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN FUNCTION ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void main(void){
    
//    // PORTA 
//    TEMP_CHANNEL_DIR    = INPUT;
//   
//    // PORTB
//    DISP1_PORT_DIR  = OUTPUT;
//    
//    // PORTD
//    DISP2_PORT_DIR = OUTPUT;
//    
//    // PORTE
//    FAN_1_DIR = OUTPUT;
//    FAN_2_DIR = OUTPUT;
//    FAN_3_DIR = OUTPUT;
//    
//    
//    // CLEARING PORT OUTPUTS
//    
//    DISP1_PORT = 0;
//    DISP2_PORT = 0;
//    PORTE = 0;
//    
//    PORTC = 0;
    
    
    LCD_Init();
//    Lcd_Cmd (LCD_CURSOR_OFF);
//    Lcd_Cmd (LCD_CURSOR_OFF);
//    Lcd_Cmd (LCD_CLEAR); 
        
//    ADC_Init();  
    
//    LCD_MESSAGE("Hola");
//    LCD_Function("Hola", "Culo");
//    LCD_Message("Hola", "Culo");
    
    sprintf(buffer, "Valor = %u", 250);
    LCD_out(buffer);
    
//    LCD_sendData('H');
    
    
    while(1){
        
        // Load a formated string on a temporal 128 bytes buffer
//        sprintf(buffer, "ADC Value  = %04lu", ADC_Read());
        
//        Lcd_Out(1, 8 - (strlen(buffer) / 2), buffer);
        
        delay_ms(500);
    }
    
    
    
    
    
    
    
    do{  // Start Infinite loop
        
        // 0.489 Is a conversion factor according to LM35 output
        // The Internal ADC step at 10bit 5v is 5v/2^10-1, which means
        // 5/1023 = 0.48875. we can approximate it to 0.489 for the LM35
        temperature = (unsigned int)(ADC_Read() * 0.489);
        
        
        
        
        
        
        
    }while(1);  // Infinite loop
    
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// AUXILIARY FUNCTIONS /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Delay in milliseconds
void delay_ms(unsigned long delay_value){
    for(unsigned long x=0; x < delay_value; x++) __delay_ms(1);
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// DISPLAYS CONVERSION FUNCTION /////////////////////////
////////////////////////////////////////////////////////////////////////////////


void ADC_Init(void){
    ADCON1bits.ADFM = 1;        // Right Justified
    ADCON1bits.ADCS2 = 1;       // RC Clock
    
    // RA0 Analog
    ADCON1bits.PCFG3 = 1;       
    ADCON1bits.PCFG2 = 1;
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG0 = 0;
    
    ADCON0bits.ADCS = 3;       // RC Clock
    
    ADCON0bits.CHS = 0;       // Channel 0 Fixed
    
    ADCON0bits.GO_DONE = 0;
    
    ADCON0bits.ADON = 1;        // ADC Module ON
}

unsigned int ADC_Read(void){
    
    unsigned int ADCConv;       // Temporal variable
    
    PIR1bits.ADIF = 0;          // Clears ADC interruption Flag (ADIF)
    delay_ms(20);               // 20mS Data Adquisition Time
    ADCON0bits.GO_nDONE = 1;    // Wait until conversion is done
    while(PIR1bits.ADIF == 0);  // Wait until conversion is done 
    
    // Conversion done
    // Save ADRESL and ADRESH Values and combines it in a single value
    // Keep in mind that ADRESL is a 8 bit register (LSB) and ADRESH is a 2 bit
    // register (MSB) from the ADC
    ADCConv = (unsigned int) (ADRESL + (ADRESH << 8));
    
    // Return ADC_CONV Value
    return ADCConv;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////// FAN CONTROL FUNCTIONS ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//void Fan_config_1(){
//    FAN_1 = ON;
//    FAN_2 = OFF;
//    FAN_3 = OFF;
//}
//
//void Fan_config_2(){
//    FAN_1 = ON;
//    FAN_2 = ON;
//    FAN_3 = OFF;
//}
//
//void Fan_config_3(){
//    FAN_1 = ON;
//    FAN_2 = ON;
//    FAN_3 = ON;
//}





