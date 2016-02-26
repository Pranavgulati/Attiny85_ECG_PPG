/*
 * Attiny85_ECG_PPG.cpp
 *
 * Created: 2/16/2016 8:38:54 PM
 * Author : Pranav
 */ 

#define F_CPU 8000000
#define NOT_A_PORT 0
#define PB 2
#define HIGH 0x1
#define LOW 0x0
#define OUTPUT 0x1
#define DELIMITER '\n'
#define SEPARATOR ','

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

static volatile uint8_t pinNo=1; //adc pin
uint8_t txPin =3;		  //Tx pin 
uint16_t bit_delay=0;
int  _tx_delay = 0;
uint8_t _transmitBitMask  = 1<<txPin;
volatile uint8_t *_transmitPortRegister;
volatile uint8_t status=1;
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
inline void tunedDelay(uint16_t delay) {
	_delay_loop_2(delay);
}
//----------------------------------------------------------------------------------------------------------------------------------
inline void pinMode(uint8_t pin, uint8_t mode){
	//pin can be 0-3 only;
	//mode 0=input 1 =output
	if(mode){//input
		DDRB=DDRB|(1<<(pin));
	}
	else{//output
		DDRB=DDRB&(~(1<<(pin)));
	}
}
//----------------------------------------------------------------------------------------------------------------------------------
inline void digitalWrite(uint8_t pin,uint8_t value){
	if(value){//LOW
		PORTB=PORTB|(1<<(pin));
	}
	else{//HIGH
		PORTB=PORTB&(~(1<<(pin)));
	}
}
//----------------------------------------------------------------------------------------------------------------------------------
uint16_t subtract_cap(uint16_t num, uint16_t sub) {
	if (num > sub)
	return num - sub;
	else
	return 1;
}
//----------------------------------------------------------------------------------------------------------------------------------
void uart_init(long speed){
	_tx_delay = 0;
	//_transmitPortRegister = (volatile uint8_t *)port_to_output_PGM[port];
	_transmitPortRegister = &PORTB;
	// Precalculate the various delays, in number of 4-cycle delays
	bit_delay = (F_CPU / speed) / 4;

	// 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
	// 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
	// 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
	// These are all close enough to just use 15 cycles, since the inter-bit
	// timings are the most critical (deviations stack 8 times)
	_tx_delay = subtract_cap(bit_delay, 15 / 4);

	tunedDelay(_tx_delay); // if we were low this establishes the end
	
	digitalWrite(txPin, HIGH);
	pinMode(txPin, OUTPUT);
}
//----------------------------------------------------------------------------------------------------------------------------------
bool uart_putchar(uint8_t b)
{//	uint8_t oldSREG = SREG;
	cli(); 
	if (_tx_delay == 0) {
		return 0;
	}

	// By declaring these as local variables, the compiler will put them
	// in registers _before_ disabling interrupts and entering the
	// critical timing sections below, which makes it a lot easier to
	// verify the cycle timings
	volatile uint8_t *reg = _transmitPortRegister;
	uint8_t reg_mask = _transmitBitMask;
	uint8_t inv_mask = ~_transmitBitMask;
	
	uint16_t delay = _tx_delay;

	 // turn off interrupts for a clean txmit

	// Write the start bit
	*reg &= inv_mask;//basically writing a 0 to the pin 
	tunedDelay(delay);

	// Write each of the 8 bits
	for (uint8_t i = 8; i > 0; --i)
	{
		if (b & 1) // choose bit
		*reg |= reg_mask; // send 1
		else
		*reg &= inv_mask; // send 0
		tunedDelay(delay);
		b >>= 1;
	}

	// restore pin to natural state
	*reg |= reg_mask;
	//SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);	
	return 1;
}
//----------------------------------------------------------------------------------------------------------------------------------
void ADC_init(){
	
//	ADMUX |= (1 << REFS0); 	// Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); 	// Left adjust ADC result to allow easy 8 bit reading
	ADMUX= (ADMUX&0xf0)|(pinNo&0x0f);	
	// Set ADC prescaler to 64 what gives 125 kHz ADC clock @ 8 MHz
	//sample rate will roughly be F_CPU/64/25 ~~4kHz
	//by default in free running mode
		ADCSRA =
		1 << ADEN |	// activate the ADC
		0 << ADSC |	// start conversion
		1 << ADATE |	// auto trigger
		0 << ADIF |	// conversion complete
		1 << ADIE |	// AD interrupt enabled
		1 << ADPS2 |	// prescaler
		1 << ADPS1 |	// prescaler
		0 << ADPS0;	// prescaler
	
	
}
void uart_putString(char inString[]){
	for(int i=0;i<3;i++){
		uart_putchar(inString[i]);
	}
	
}

//----------------------------------------------------------------------------------------------------------------------------------
ISR(ADC_vect){
	cli();
//void ADCin(){
		//uart_putchar((char)pinNo);
		digitalWrite(2,status%2);
		status++;
		uint8_t low,high;
		low =ADCL;
		high=ADCH;
		low++;
		//char highString[3];
		//{
			//highString[0]=(high/100)+48;
			//high=high%100;
			//highString[1]=(high/10)+48;
			//high=high%10;
			//highString[2]=high+48;
			//
		//}
		
		if(pinNo==1){
			pinNo=2;
			uart_putchar(high);
			//uart_putchar(low);//
			uart_putchar(SEPARATOR);
		}
		else if(pinNo==2){
			pinNo=1;
			uart_putchar(high);
			//uart_putchar(low);
			uart_putchar(DELIMITER);
			
		}
		ADMUX= (ADMUX&0xf0)|(pinNo);		//selecting the required pin
		sei();
		ADCSRA |= (1 << ADSC);//restart conversion
//		ADCSRA&=(~(1<<ADIF));
		
}
	


int main (void)
{	
	ADC_init();
	uart_init(115200);
	sei();
	ADCSRA |= (1 << ADSC);//restart conversion
	
	
   //which pin is to be used 
   pinMode(2,OUTPUT);
   	while(1)
	{

	}
}


/*
if ((ADCSRA & (1<<ADIF))!=0){
ADCin();
uart_putchar('a');
}
else{
uart_putchar('d');
}
*/