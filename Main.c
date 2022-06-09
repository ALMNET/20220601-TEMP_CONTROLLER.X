////////////////////////////////////////////////////////////////////////////////

/*
 * File:   Main.c
 * Author: ALMNET
 *
 * Created on Jun 3, 2022, 16:14
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

#define PWM_PERIOD_US   100
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
unsigned int temperature;           // Read temperature from PT100 / LM35

unsigned long pwmDutyValue = 70;    // Duty Cycle, from 0 to 100 (0 to 100%)
unsigned long pwmDutyCnt = 0;       // Duty counter (increments until reach pwmDutyValue)
unsigned long pwmPeriod = 0;        // pwm Period counter

unsigned long tmr2Counter = 0;      // Counter to enable or disable the tmr2 interrupt
                                    // if this counter is 0, the tmr2 will be disabled
                                    // otherwise it will be enabled.

char buffer[64];                    // Buffer for lcd messages

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


///////////////////////////////////////////////////////////////////////////////
////////////////////////////// INTERRUPT HANDLING /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void interrupt ISR()
{
    if(TMR2IE == 1 && TMR2IF == 1){
        
        TMR2IF = 0;
        
        if(tmr2Counter)
        {
//            tmr2Counter--;
            if(pwmPeriod < PWM_PERIOD_US)
            {
                if(pwmDutyCnt < pwmDutyValue)
                {
                    SERVO = 1;

                    pwmDutyCnt++;
                }

                else{
                    SERVO = 0;
                }

                pwmPeriod++;
            }
            else
            {
                pwmPeriod = 0;
                pwmDutyCnt = 0;
                tmr2Counter--;
            }
        }
        
    }
    return;
}


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN FUNCTION ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void main(void){
    

    TEMP_CHANNEL_DIR    = INPUT;
    SERVO_DIR = OUTPUT;
    
    ADC_Init(1);
    LCD_Init();
    
    tmr2_init();
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

/**
 * @brief       General Delay function
 *              This function uses internal core cycles to create a delay
 *   
 * @param[in]   delay_value: 
 *              32 bit value which sets the desired delay value in millisecs
 *              for example, if delay_value = 5400 it means the delay
 *              will be of 5400 millisecs or 5.4 secs
 * 
 * @return      None (void type function)
 */
void delay_ms(unsigned long delay_value){
    for(unsigned long x=0; x < delay_value; x++) __delay_ms(1);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// ADC FUNCTIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief       Initializes the internal ADC Module
 *   
 * @param[in]   adcEnabledChannels: 
 *              Number of analog channels to enable. In this case 4 is enough
 *              for our requirement
 * 
 * @return      None (void type function)
 */

void ADC_Init(const unsigned char adcEnabledChannels){
    
    // Channel enabler / decoder (Configured to use up to 4 analog channels)
    const unsigned char PCFG_DECODER[5] = {0, 0b1110, 0b0101, 0b0100, 0b0011}; 
    
    // Enables the number of required analog channels
    ADCON1bits.PCFG = PCFG_DECODER[adcEnabledChannels];
    
    ADCON1bits.ADFM = 1;        // ADC ADDRESS Right Justified
    
    ADCON1bits.ADCS2 = 1;       // RC Clock
    ADCON0bits.ADCS = 3;        // RC Clock
    
    ADCON0bits.GO_DONE = 0;     // Conversion off
    
    ADCON0bits.ADON = 1;        // ADC Module ON
}

/**
 * @brief       Reads the ADC channel according to parameter
 *              
 * @param[in]   adcChannel: 
 *              Analog channel to read
 * 
 * @return      10 bit ADC value result from conversion
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
//////////////////////// PWM CONFIG OVER TMR2 INTERRUPT ////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief       Configures the TMR2 Module for PWM output
 *              Through interrupt
 *   
 * @param[in]   None
 * 
 * @return      None
 */

void tmr2_init()
{
  T2CON	     = 0x04;
  PR2        = 199;
  TMR2IE     = 1;
  INTCON     = 0xC0;
  
  INTCONbits.GIE = 1;
  INTCONbits.PEIE = 1;
  
  PIE1bits.TMR2IE = 1;
  
  SERVO_DIR = OUTPUT;
}

/**
 * @brief       Loads TMR2 values according to desired servomotor behavior
 *              Basically it enables the square 50 hz signal and using the
 *              parameters, we can set the number of steps (movements) and
 *              the size or lenght of the movement (degrees)
 *   
 * @param[in]   steps: The number of steps we required
 * 
 * @return      stepSize: The size of the steps (degrees). The value needs to
 *              be between 0 and 90�, otherwise the function will return an
 *              error
 * 
 * @return      0: If the operation was sucessful
 *              1: If the stepSize value is beyond the range
 *              2: If the steps value is equal to 0 (No operation executed)
 */

unsigned char servo_loader(unsigned int steps, unsigned int stepSize)
{
    unsigned char result = 0;       // Asumes no error by default
    
    if(steps)
    {
        if(stepSize > 0 && stepSize <= 90)
        {
            tmr2Counter = steps;
            //tmr2Counter = steps * PWM_PERIOD_US;
            pwmDutyValue = stepSize * 100 / 90 ;
                    
        }
        
        else
            result = 1;             // stepSize beyond the allowed range
                                    // function returns with a type 1 error
        
    }
    
    else
        result = 2;                 // steps value equal to 0
                                    // function returns with a type 2 error
    
    return result;                  // return from call
    
    
}

