/*
 * Attiny85_ECG_PPG.cpp
 *
 * Created: 2/16/2016 8:38:54 PM
 * Author : Pranav
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
volatile int PPG_in;
volatile int ECG_in;
void ADCinit(){
	
	ADMUX |= (1 << REFS0); 	// Set ADC reference to AVCC
	ADMUX |= (1 << REFS1); 	// Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); 	// Left adjust ADC result to allow easy 8 bit reading
	ADCSRA |= (1 << ADPS2);	// Set ADC prescaler to 32 what gives 31.25 kHz sample rate @ 1 MHz
	ADCSRA |= (1 << ADPS0);
	//ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA|=(1<<ADATE);    //auto retriggering enabled
	//by defautl in free running mode
	
}
int analogRead(uint8_t pin){
ADMUX|=(pin&0x0f);		//selecting the required pin	
ADCSRA |= (1 << ADSC);  // Start ADC Conversion
uint8_t low,high;
low =ADCL;
high=ADCH;
return (high<<8|low);
}
int main (void)
{	ADCinit();
   uint8_t ecgPin=PINB1;
   uint8_t ppgPin=PINB2;
   analogRead(ecgPin);
   analogRead(ppgPin);
   //which pin is to be used 
	while(1)
	{
		
	}
}



