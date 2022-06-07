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
//#define _XTAL_FREQ 500000

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

#define PWM_PERIOD_US   200000

//////////////////////////////////// INPUTS ////////////////////////////////////

// PORTA 
#define TEMP_CHANNEL_DIR    TRISA0
#define TEMP_CHANNEL        RA0

/////////////////////////////////// OUTPUTS ////////////////////////////////////

// SERVO
#define SERVO_DIR           TRISC2
#define SERVO               RC2
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// VARIABLES  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Temperature value obtained from ADC Read
unsigned int temperature;

unsigned long pwmDutyTmr2 = 5000;
unsigned long pwmDutyCnt = 0;
unsigned long pwmPeriod = 0;

char buffer[64];

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// PROTOTYPES FUNCTIONS  /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// General delay function
void delay_ms(unsigned long delay_value);

// ADC Functions
void ADC_Init(const unsigned char adcEnabledChannels);
unsigned int ADC_Read(unsigned char adcChannel);

// PWM Functions for servomotor control
void PWM_Init();
void PWM_Update(unsigned int PWMDuty);
void tmr2_init();





void interrupt ISR()
{
    if(TMR2IE == 1 && TMR2IF == 1){
        TMR2IF = 0;
        
//unsigned long pwmDutyTmr2 = 0;
//unsigned long pwmDutyCnt = 0;
//unsigned long pwmPeriod = 0;
//        #define PWM_PERIOD_US   20000
        
        
        if(pwmPeriod < PWM_PERIOD_US)
        {
            if(pwmDutyCnt < pwmDutyTmr2)
            {
                if(!SERVO) SERVO = 1;
                pwmDutyCnt++;
            }
            
            SERVO = 0;
            
            pwmPeriod++;
        }
        else
        {
            pwmPeriod = 0;
            pwmDutyCnt = 0;
        }
        
    }
    
    
    
    
    return;
}
















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
    
    tmr2_init();
    
//    PWM_Init();
    
    
    
//    LCD_sendData('H');
    
    
    while(1){
        
        // Load a formated string on a temporal 128 bytes buffer
//        sprintf(buffer, "ADC Value  = %04lu", ADC_Read());
        
//        Lcd_Out(1, 8 - (strlen(buffer) / 2), buffer);
        
        temperature = (unsigned int) ADC_Read(TEMP_CHANNEL);
        
        PWM_Update(temperature);
        
        sprintf(buffer, "Valor = %04u", temperature);
        LCD_out(1, 1, buffer);
        
        delay_ms(100);
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
 * @brief       Reads the ADC channel according to parameter
 *              
 *   
 * @param[in]   adcChannel: 
 *              Analog channel to read
 * @param[in]   pos_x: 
 *              x position coord. must be between 1 to 16, otherwise the 
 *              function will return an error (1)
 * @param[in]   lcd_msg: 
 *              Pointer / Array to the message to send. If the message
 *              is empty, the function will return 2
 * 
 * @return      10 bit 
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
/////////////////////////////////// PWM INIT ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief       Initializes the PWM Module for the servo
 *              
 * @param       None 
 * 
 * @return      None 
 */

void PWM_Init(){
    // 1.- Disable the PWM pin (CCPx) Outputdriver as an input
    //     by setting the associated TRIS Bit
    TRISC2 = INPUT;
    
    // 2.- Set the PWM period by loading the PR2 Register
    PR2 = 0b10011011;
    
    // 3.- Configure the CCP module for the PWM mode by loading
    //     the CCPxCON register with the appropiate values
    CCP1CON = 0b00111100; // duty lowest bits + PWM mode
    
    // 4.- Set the PWM duty cycle by loading the CCPRxL register
    //     and DCxB<1:0> bits of the CCPxCON register
    CCPR1L = 0b00000111;  // set duty MSB
    
    // 5.- Configure and start Timer 2:
    //     * Clear the TMR2IF interrupt flag of the PIR1 Register
    TMR2IF = 0;
    
    //     * Set the TIMER2 Prescale value by loading the T2CKPS 
    //       bits on the T2CON register and enable it by setting
    //       the TMR2ON bit on the T2CON register
    T2CON = 0b00000111; // prescaler + turn on TMR2;
    
    // 6.- Enable PWM output after a new PWM cycle has started
    //      * Wait until Timer2 overflows (TMR2IF bit of the PIR1 reg)
    while(TMR2IF == 0);
    
    //      * Enable the CCPx pin output driver by clearing the 
    //        associated TRIS bit
    TRISC2 = OUTPUT;
}

/**
 * @brief       Updates the PWM duty cycle
 *              
 *   
 * @param[in]   PWMDuty: 
 *              PWM Duty Cycle value
 * 
 * @return      None
 */

void PWM_Update(unsigned int PWMDuty){
    unsigned int PWM_L, PWM_H;
    
    double pulseWidth;

    unsigned long PWMValue = 0;
    
    // Pulse Width value (From 1 to 1+0.99)
    pulseWidth = 1 + (PWMDuty * 0.0009765);
	
    // PWM Value, where 31 is for 1 mS (5%) and 62 is for 2 mS (10%)
	PWMValue = (unsigned long ) (31 * pulseWidth);
    
    // Rotates PWM Value 2 bits and save it into PWM_H
    PWM_H = PWMValue >> 2;
    
    // Clears 2 PWM LSBs
    PWM_L = (unsigned int) (CCP1CON & 0b11001111);
    
    // Filters 6 More significant bits from PWM Value and save it itself
    PWMValue &= 0b00000011;
    
    // PWM_VALUE Rotates itself 4 bits to left (To save into CCP1CON 5:4 bits)
    PWMValue = PWMValue << 4;
    
    // Saves it into PWM_L (Which will be saved on CCP1CON to update PWM
    // Pulse Witdh)
    PWM_L = PWMValue + PWM_L;
    
    // Updating Values
    CCP1CON = PWM_L;
    CCPR1L  = PWM_H;
    
}






void tmr2_init()
{
  T2CON	     = 0x04;
  PR2        = 9;
  TMR2IE     = 1;
  INTCON     = 0xC0;
  
  INTCONbits.GIE = 1;
  INTCONbits.PEIE = 1;
  
  PIE1bits.TMR2IE = 1;
  
  SERVO_DIR = OUTPUT;
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





