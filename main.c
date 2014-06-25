#include <xc.h>
#include <stdio.h>          // library containing printf() function
#include <spi.h>

#include "configureUSART.h" // library for configureUSART(baud)

#pragma config WDT = OFF
#pragma config OSC = INTIO7      // puts osc/4 on pin 14 to check freq
#pragma config MCLRE = OFF
#pragma config LVP = OFF
#pragma config PBADEN = OFF      // PORTB<4:0> are digital IO 


void WaitOneSecond(void);
void set_DAC_command( unsigned int );
void command_DAC(void);

void set_osc_32MHz(void);

union command_dac_u
{
    unsigned int value;
    struct
    {
        unsigned valueLower: 8;
        unsigned valueUpper :4;
        unsigned output_shutdown_control:1;
        unsigned gain_select:1;
        unsigned buffer_control:1;
        unsigned ignore_command:1;
    };
    struct
    {
        unsigned lowerByte: 8;
        unsigned upperByte: 8;
    };
} command_dac = 0x7800;
    
     

int main(void)  // a C project can only have one main() function
{

    set_osc_32MHz(); // set MCU to run at 8 MHz
                     

    configureUSART(9600ul, 8); // configure MCU serial communication module to run at 9600 baud 
                                // with MCU operating at 32 MHz. Defined in configureUSART.c
                        // 9600 bits/s is default communication rate
                        // other choices 2 400, 4 800, 19 200, 38 400 (not available at 1 MHz)
                                // LCD will not work if MCU is not configured for USART    

 
 
    printf("DAC 12 bit /r/n");
    

    
    TRISCbits.TRISC0 = 0;
    PORTCbits.RC0 = 1;
    OpenSPI( SPI_FOSC_4, MODE_00, SMPEND); 

    set_DAC_command( 0xfff/5*1 );
    command_DAC();


    while(1)
    {

    }    
    return 0;
}

void set_DAC_command( unsigned int value )
{
    command_dac.value =  value;
    command_dac.ignore_command = 0;
    command_dac.buffer_control = 1;
    command_dac.gain_select = 1;
    command_dac.output_shutdown_control = 1;
}    

void command_DAC ( void )
{
    PORTCbits.RC0 = 0;
    WriteSPI(command_dac.upperByte);
    WriteSPI(command_dac.lowerByte);
    PORTCbits.RC0 = 1;
} 

void set_osc_32MHz(void)
{
  int i;
 
  OSCCONbits.IRCF2 = 1;     // Set the OSCILLATOR Control Register to 8 MHz
  OSCCONbits.IRCF1 = 1;      
  OSCCONbits.IRCF0 = 1;     
 
  OSCTUNEbits.PLLEN = 1;    // Enable PLL, boost by 4 -> 32 MHz

  for(i=0;i<500;i++);       // delay to allow clock PLL to lock (stabilize)

      
}   
