/*
 * IRSensor.c
 *
 * Created: 2014/08/08
 *  Author: sapporo62
 */ 

#include "IRSensor.h"

void initIRSensor() {
	
	DDRA  = 0x7F;	// PORTA as input
	PORTA = 0x00;	// Enable the pull-up on PORTA7
	
	DDRF  = 0x00;	// PORTF as input
	PORTF = 0xff;	// Enable the pull-up on PORTF all
	
	ADMUX = ADC_PORT_1;		// ADC Port 1 Select
	ADCSRA |= (1 << ADLAR);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	// Clock 1/64div.
	ADCSRA |= (1 << ADEN);	// ADC Enable
}

unsigned int ReadIRSensor(unsigned int ch)
{
	ADMUX = ch;

	//Start Single conversion
	ADCSRA |= (1 << ADIF);
	ADCSRA |= (1 << ADSC);
	
	//Wait or conversion to complete
	while( ! (ADCSRA & (1<<ADIF))) ;
	
	//Clear ADIF by writing one to it, the loop ends when ADIF is set to 1.
	//The conversion is now complete
	
	return(ADC);
	
}
