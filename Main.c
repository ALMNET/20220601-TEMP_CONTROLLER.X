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

void ADC_Init(const unsigned char adcEnabledChannels);
unsigned int ADC_Read(unsigned char adcChannel);



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
    
    ADC_Init(1);
    LCD_Init();
    
    
    
//    LCD_sendData('H');
    
    
    while(1){
        
        // Load a formated string on a temporal 128 bytes buffer
//        sprintf(buffer, "ADC Value  = %04lu", ADC_Read());
        
//        Lcd_Out(1, 8 - (strlen(buffer) / 2), buffer);
        
        sprintf(buffer, "Valor = %04u", ADC_Read(0));
        LCD_out(1, 1, buffer);
        
        delay_ms(500);
    }
    
    
    
    
    
    
    
    do{  // Start Infinite loop
        
        // 0.489 Is a conversion factor according to LM35 output
        // The Internal ADC step at 10bit 5v is 5v/2^10-1, which means
        // 5/1023 = 0.48875. we can approximate it to 0.489 for the LM35
        temperature = (unsigned int)(ADC_Read(TEMP_CHANNEL) * 0.489);
        
        
        
        
        
        
        
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
////////////////////////////////// ADC FUNCTIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

void ADC_Init(const unsigned char adcEnabledChannels){
    
    // Channel enabler / decoder (Configured to use up to 4 analog channels)
    const unsigned char PCFG_DECODER[5] = {0, 0b1110, 0b0101, 0b0100, 0b0011}; 
    
    // Enables the number of required analog channels
    ADCON1bits.PCFG = PCFG_DECODER[adcEnabledChannels];
    
    ADCON1bits.ADFM = 1;        // ADC ADDRESS Right Justified
    ADCON1bits.ADCS2 = 1;       // RC Clock
    
    // RA0 Analog
//    ADCON1bits.PCFG3 = 1;       
//    ADCON1bits.PCFG2 = 1;
//    ADCON1bits.PCFG1 = 1;
//    ADCON1bits.PCFG0 = 0;
    
    ADCON0bits.ADCS = 3;       // RC Clock
    
//    ADCON0bits.CHS = 0;       // Channel 0 Fixed
    
    ADCON0bits.GO_DONE = 0;
    
    ADCON0bits.ADON = 1;        // ADC Module ON
}

/**
 * @brief       Reads the 
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

unsigned int ADC_Read(unsigned char adcChannel){
    
    unsigned int ADCConv;       // Temporal variable
    
    ADCON0bits.CHS = adcChannel;
    
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





