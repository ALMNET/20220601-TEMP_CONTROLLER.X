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


#define TEMP_CHANNEL_DIR    TRISA0
#define TEMP_CHANNEL        RA0


#define IN_IR_DIR           TRISB1
#define IN_IR               RB1

#define OUT_IR_DIR          TRISB2
#define OUT_IR              RB2

#define BUTTON_INC_DIR      TRISB3
#define BUTTON_INC          RB3

#define BUTTON_DEC_DIR      TRISB4
#define BUTTON_DEC          RB4

#define BUTTON_SET_DIR      TRISB5
#define BUTTON_SET          RB5

/////////////////////////////////// OUTPUTS ////////////////////////////////////

// SERVO
#define SERVO_DIR           TRISC2
#define SERVO               RC2
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// VARIABLES  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Temperature value obtained from ADC Read
float temperature;                  // Read temperature from PT100 / LM35

unsigned long pwmDutyValue = 0;    // Duty Cycle, from 0 to 100 (0 to 100%)
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

// ADC Prototypes
void ADC_Init(const unsigned char adcEnabledChannels);
unsigned int ADC_Read(unsigned char adcChannel);

// PWM Prototypes
void tmr2_init();
unsigned char servo_loader(unsigned long steps, unsigned char stepSize);


///////////////////////////////////////////////////////////////////////////////
////////////////////////////// INTERRUPT HANDLING /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void interrupt ISR()
{
    if(TMR2IE == 1 && TMR2IF == 1){
        
        TMR2IF = 0;
        
        if(tmr2Counter)
        {
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
    
    // Configuring Inputs
    TEMP_CHANNEL_DIR    = INPUT;
    
    IN_IR_DIR           = INPUT;
    OUT_IR_DIR          = INPUT;
    
    BUTTON_INC_DIR      = INPUT;
    BUTTON_DEC_DIR      = INPUT;
    BUTTON_SET_DIR      = INPUT;
            
    // Configuring Outputs
    SERVO_DIR = OUTPUT;
    
    ADC_Init(1);        // Initializing Internal ADC Module
    LCD_Init();         // Initializing LCD (I/O, Bus width and operation modes)
    
    tmr2_init();        // Initializing and configuring tmr2 (100 uS overflow)
         

    // TODO: Delete this section
    // Call servo loader for servo testing on simulation.
    servo_loader(200, 45);
    
    while(1){
        
        // Refer to attached excel tables for reference about Zero and Span
        // Values.
        
        // Zero compensated, Because on 100 ohm 0ºC, the read value is 24, so
        // in that way we force the value to 0 at 100 ohm, 0ºC)
        temperature = ADC_Read((unsigned char)TEMP_CHANNEL) - 24;
        
        // Span compensation
        temperature *= 0.277141877;
        
        // Save the formated string in the buffer array
        sprintf(buffer, "Valor = %2.2f", temperature);
        
        // Message out through LCD
        LCD_out(1, 1, buffer);
        
        // 100 ms refresh delay (10 Frames per sec aprox)
        delay_ms(100);
        
    }   // Infinite loop
    
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
  
  INTCONbits.GIE = 1;       // Global Interrupt Enable
  INTCONbits.PEIE = 1;      // Peripheral Interrupt Enable
  
  PIE1bits.TMR2IE = 1;      // TMR2 specific Interrupt Enable
  
  SERVO_DIR = OUTPUT;       // Servo pin as output
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
 *              be between 0 and 90º, otherwise the function will return an
 *              error
 * 
 * @return      0: If the operation was sucessful
 *              1: If the stepSize value is beyond the range
 *              2: If the steps value is equal to 0 (No operation executed)
 */

unsigned char servo_loader(unsigned long steps, unsigned char stepSize)
{
    unsigned char result = 0;       // Asumes no error by default
    
    if(steps)
    {
        if(stepSize > 0 && stepSize <= 90)
        {
            tmr2Counter = steps;
            pwmDutyValue = (unsigned long) stepSize * 10 / 90 ;
                    
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

