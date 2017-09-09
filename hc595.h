/*

Some of the HC595 code was found here and was modified for this purpose:
http://extremeelectronics.co.in/avr-tutorials/using-shift-registers-with-avr-micro-avr-tutorial/
2017, B. Redemann
*/

#ifndef _HC595_H
#define _HC_595_H

#define HC595_PORT   PORTB    // Port B is used
#define HC595_DDR    DDRB

#define HC595_DS_POS PB0      // Data pin (DS Pin14 DIP) pins
#define HC595_SH_CP_POS PB1   // Shift clock (SH_CP) pins
#define HC595_ST_CP_POS PB2   // Store clock (ST_CP) pins

#define HC595DataHigh() (HC595_PORT|=(1<<HC595_DS_POS))   //Low level macros to change data (DS)lines
#define HC595DataLow() (HC595_PORT&=(~(1<<HC595_DS_POS)))

void hc595_init(void);
void hc595_pulse(void);
void hc595_latch(void);
void hc595_write(uint16_t data);

//Initialize HC595
void hc595_init(void)
{
   //Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
   HC595_DDR|=((1<<HC595_SH_CP_POS)|(1<<HC595_ST_CP_POS)|(1<<HC595_DS_POS));
}

//Sends a clock pulse on SH_CP line
void hc595_pulse(void)
{
   //Pulse the Shift Clock
   HC595_PORT|=(1<<HC595_SH_CP_POS);// HIGH
   HC595_PORT&=(~(1<<HC595_SH_CP_POS));// LOW
}

//Sends a clock pulse on ST_CP line
void hc595_latch(void)
{
   //Pulse the Store Clock
   HC595_PORT|=(1<<HC595_ST_CP_POS);// HIGH
   _delay_loop_1(1);
   HC595_PORT&=(~(1<<HC595_ST_CP_POS));// LOW
   _delay_loop_1(1);
}

void hc595_write(uint16_t data)
{
   // Send each 16 bits serially, we have two HC595
   // Order is MSB first
   for(uint16_t i=0;i<16;i++)
   {
      // Output the data on DS line according to the Value of MSB
      if(data & 0x8000)   // = binary 1000000000000000
      {
		HC595DataHigh();  // MSB is 1 so output high
      }
      else
      {
        HC595DataLow();  // MSB is 0 so output high
      }
 
      hc595_pulse();      // Pulse the clock line
      data=data<<1;       // Now bring next bit at MSB position

   }

   //Now all 16 bits have been transferred to shift register
   //Move them to output latch at one
   hc595_latch();
}

#endif