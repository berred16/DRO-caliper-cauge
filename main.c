/*
Copyright (C) 2017  Bernhard Redemann (b-redemann@gmx.de)
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This program reads out the measured data from a "TOP Craft" digital caliper
gauge and sends it over uart to the PC. UART: 9600, 8N1. 
Also a 7-Segment display is connected via HC595 shift register.

Features:
Sending data to uart is set by jumper at PC0
If jumper is set, uart send every 3 second the data
CA or CC deisplays can be used

Some of the HC595 code was found here and was modified for this purpose:
http://extremeelectronics.co.in/avr-tutorials/using-shift-registers-with-avr-micro-avr-tutorial/


Version 0.7 (q+d) / 2017, Aug. 38th


Hardware setup:

Atmega 8 / DIP, 28 pol
	ISP, Reset		|1-----U-----28| 
			RxD		|2			 27|
			TxD		|3		     26|
INT0 GATE	-> PD2	|4			 25|
INT1 CAL_CLK-> PD3	|5			 24| PC1, CC or CA
CAL_DATA 	-> PD4	|6			 23| PC0, uart j/n
	VCC				|7			 22| GND
	GND				|8			 21| AREF
					|9			 20| AVCC
					|10			 19| SCK, ISP
					|11			 18| MISO, ISP
					|12			 17| MOSI, ISP
					|13			 16| PB2 <- RCK 595
	SER 595 -> PB0	|14----------15| PB1 <- SCK 595

Fuses: 8MHz intern
*/

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include "hc595.h"

#define F_CPU 8000000UL

/* For the general datapacket of the caliper */
unsigned int data[24]={1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,32768,0,0,0,1,0,0,0,1};
// 24 Bits with weighting (16 bit value), last bit: inch/mm, bit 20: +/-

unsigned int datapulse[24]; // For 24 pulses inside the frame

volatile unsigned int gate, gi; // Gate variables
volatile unsigned int datavalue, datavalue_old, first_inch; // The measured value
volatile unsigned int digit0, digit1, digit2, digit3, digit4, digit5; // Single digits
volatile unsigned int ci, ii;

int unit1, unit2, sign, digit_sign; // Unit and +/- sign

int i, n, ui; // Counter variables (ui=uart i)

/* For the HC595 and the 7-Segement displays */
#define AN1 1	// All anodes 
#define AN2 2
#define AN3 4
#define AN4 8
#define AN5 16
#define AN6 32


/* 7-Segment display pattern (invers, because of common anodes) 
/ Setup:
/  dpgfedcbaxxANODES 1-6
/  	||||||||  ||||||
/ 0b1100000000000000 
*/

// For CC displays
volatile uint16_t led_pattern[13]={
                        0xC000, // -> 0b1100000000000000 0 3FFF (Cothode)
                        0xF900, // -> 0b1111100100000000 1 06FF
                        0xA400, // -> 0b1010010000000000 2 5Bff
                        0xB000, // -> 0b1011000000000000 3 4FFF
                        0x9900, // -> 0b1001100100000000 4 66FF
                        0x9200, // -> 0b1001001000000000 5 6DFF
						0x8200, // -> 0b1000001000000000 6 7DFF
						0xF800, // -> 0b1111100000000000 7 07FF
						0x8000, // -> 0b1000000000000000 8 7FFF
						0x9000, // -> 0b1001000000000000 9 6FFF
						0x7F00, // -> 0b1111111000000000 decimal point 80FF
						0xBF00,  // -> 0b1011111100000000 minus sign 40FF
						0xFF00  // -> off 00FF
                     };

// For CA displays					 
/*volatile uint16_t led_pattern[13]={
                        0x3FFF, 
                        0x06FF, 
                        0x5Bff, 
                        0x4FFF, 
                        0x66FF, 
                        0x6DFF, 
						0x7DFF, 
						0x07FF, 
						0x7FFF, 
						0x6FFF, 
						0x80FF, 
						0x40FF, 
						0x00FF 
                     };*/
					 


char uart_init(void) {
    UCSRB |= ((1<<RXEN)|(1<<TXEN)); // Enable rx and tx
	UCSRC |= ((1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)); // Set asynchron, no parity, 1 stopbit, 8 databits 
	UBRRL = 51; // Set baudrate to 9600 at 8Mhz (0.2% error)
	return 0;
	} 

char calc_digits(void) {
		switch(datapulse[23]) // check if inch or mmm
			{
				case 0: // Calculate and send for mm
				{
				digit0 = digit_sign;
				digit1 = (datavalue/10000); // +48 for uart (ascii); 		
				digit2 = ((datavalue%10000)/1000);
				digit3 = (((datavalue%10000)%1000)/100); //&led_pattern[10]; // & dp for 7-segment display (for uart: UDR=44)
				digit4 = ((((datavalue%10000)%1000)%100)/10);
				digit5 = ((((datavalue%10000)%1000)%100)%10);
				}
				break;
			
				case 1: // Calculate and send for in
				{
				first_inch = datapulse[0]; // save first bit
				datavalue >>=1;	// Shift ddatavalue to the right, another weighting for inches
				
                digit0 = digit_sign;
				digit1 = ((datavalue%10000)/1000);// // +48 for uart (ascii); 		
				digit2 = (((datavalue%10000)%1000)/100);
				digit3 = ((((datavalue%10000)%1000)%100)/10);//&led_pattern[10]; // & dp for 7-segment display (for uart: UDR=44)
				if (first_inch == 1)		// there is still an unknown problem with that ...
				digit4 = (((datavalue-1%10000)%1000)%100)%10; //+48; 
				else 
				digit4 = (((datavalue%10000)%1000)%100)%10; //+48;
				digit5 = first_inch * 5; //((((datavalue%10000)%1000)%100)/10);
				}
				break;
		
	}
			
	return 0;
	}
	
ISR(INT0_vect) // Gate, alway before data frame
	{
			//ii++;
			//if (ii > 1) { // only every 2.nd interrupt
				gate = 1;		// Gate active
				datavalue = 0;  // All data to 0
			//	ii=0;
			//	}
	}		

ISR(INT1_vect) // Clocksignal (24x inside the gate)
{
	if (gate == 1) // Gate is set (start point)
	{
					if (PIND & (1 << PD4) ) // Check PD4 if high
						{
						datapulse[gi] = 1; // Set datapulse to 1
						}
					else
						{
						datapulse[gi] = 0; // Set datapulse to 0
						}
			
					if (gi <24) // Just do this 24 times (for 24 bits, 0-23)
						{
							{
							datavalue = datavalue + (data[gi]*datapulse[gi]); // Sum of all single bits
							}
						gi ++;
						}
						
					if (datapulse[20] == 1)
						{
						sign = 1; //45; // Show "-"
						digit_sign = 11; 
						}
						
					if (datapulse[20] == 0)
						{
						sign = 0; //43; // Show "+"	
						digit_sign = 12;
						}
					// End
			
					if (gi == 24) // Reset to 0, no more pulses
						{
						gate = 0;
						gi = 0;
						}
	}	
		
	else // Gate = 0, no gate connected
		{
		// Maybe "error gate" message,if gate signal is not there ...
		}
}					 

int main(void)
{

DDRC = (0 << PC0); // PC0 as input / jumper to set uart y/n

// Configure external interrupts INT0 and INT1
GICR |= ((1 << INT0)|(1 << INT1)); // Enable INT0 and INT1
MCUCR |= ((1 << ISC01)|(1 << ISC00)|(1 << ISC11)|(1 << ISC10)); // Set both to rising edge

hc595_init();

uart_init();

sei();

while(1)
{
	
	
		{
		if (gate == 0)
			{
			
			calc_digits();
			
			switch(datapulse[23])
			{
				case 0: // mm 
				{		
					for(int i=0;i<431;i++) // i just tested, 431 is o.k. (less flickering)
						{
						
						hc595_write(led_pattern[digit0]+AN1);
						_delay_us(7); // 7 just tested, is o.k.
						hc595_write((led_pattern[digit1])+AN2); 
						_delay_us(7);
						hc595_write(led_pattern[digit2]+AN3);
						_delay_us(7);
						hc595_write((led_pattern[digit3]&led_pattern[10])+AN4); // dp at position digit 4
						_delay_us(7);
						hc595_write(led_pattern[digit4]+AN5);
						_delay_us(7);
						hc595_write(led_pattern[digit5]+AN6);
						_delay_us(7);
						
						if (PINC & (0 << PC0) ) {
							ui ++;
							if (ui > 10000)	{
										while ( !( UCSRA & (1<<UDRE)) );
										UDR = sign; 						// Print sign
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit1+48; 		// +48 for ASCII characters
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit2+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit3+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = 44; 							// 44 = ","
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit4+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit5+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = 109; 							// "m"
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = 109; 							// "m"
										
										ui = 0;
							}
						}
					}
				}
				break;
			
				case 1: // in 			
				{
					for(int i=0;i<431;i++) // i just tested, 431 is o.k. (less flickering)
						{
						
						hc595_write(led_pattern[digit0]+AN1);
						_delay_us(7); // 7 just tested, is o.k.
						hc595_write((led_pattern[digit1]&led_pattern[10])+AN2); 
						_delay_us(7);
						hc595_write((led_pattern[digit2])+AN3);
						_delay_us(7);
						hc595_write((led_pattern[digit3])+AN4); // dp at position digit 2	
						_delay_us(7);
						hc595_write(led_pattern[digit4]+AN5);
						_delay_us(7);
						hc595_write(led_pattern[digit5]+AN6);
						_delay_us(7);
						
						if (PINC & (0 << PC0) ) {
							ui ++;
							if (ui > 10000)	{
										while ( !( UCSRA & (1<<UDRE)) );
										UDR = sign; 						// Print sign
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit1+48; 		// +48 for ASCII characters
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = 44; 							// 44 = ","
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit2+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit3+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit4+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = digit5+48;
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = 105; 							// "i"
										while ( !( UCSRA & (1<<UDRE)) ) ;
										UDR = 110; 							// "n"
										
										ui = 0;
										}
						}
					}
				}
				break;
				
			}
		
		}
		
	}	
}	
	
return 0;
}
