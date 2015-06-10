/*
 * SensorManager.c
 *
 * Created: 2014/08/19 18:56:34
 *  Author: Administrator
 */ 

#include "SensorManager.h"

#define IR_SIZE ADC_PORT_6 + 1
#define DBG 0

void timer_init(void);

unsigned int IRArrays[IR_SIZE];

void initIRSensor(void) {
    
    DDRA  = 0xFF;   // PORTA as output
    PORTA = 0x00;   // Enable the pull-up on PORTA7
    
    DDRF  = 0x00;   // PORTF as input
    PORTF = 0xff;   // Enable the pull-up on PORTF all
    
    ADMUX = ADC_PORT_1;     // ADC Port 1 Select
    ADCSRA |= (1 << ADLAR);
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);  // Clock 1/64div.
    ADCSRA |= (1 << ADEN);  // ADC Enable
    
//  timer_init();
//  sei();
}

unsigned int ReadIRSensor(unsigned int ch) {
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

void ReadIRSensors(unsigned int * sensors) {
    unsigned int i = 0;
    for (i = ADC_PORT_1; i <= ADC_PORT_6; i++) {
        sensors[i] = ReadIRSensor(i);
    }
    return;
}

void timer_init(void) {
    TCCR1B = 0x00;  // stop
    TCNT1 = 0x00;   // set count
    OCR1A = 1;   // 1us * 1024 -> 1.024ms
    TCCR1A = (1 << WGM01);  // CTC mode
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B = (1 << CS12) | (1 << CS10);  // start timer Clock/1024
}

ISR(TIMER1_COMPA_vect) {
    // Update Sensors
    unsigned int * IR = NULL;
    
    ReadIRSensors(IRArrays);
    if (DBG) {
        printf( "PORT_1:%d\r\n", IR[ADC_PORT_1]);
        printf( "PORT_2:%d\r\n", IR[ADC_PORT_2]);
        printf( "PORT_3:%d\r\n", IR[ADC_PORT_3]);
        printf( "PORT_4:%d\r\n", IR[ADC_PORT_4]);
    }
}

void getIRSensors(unsigned int * array) {
    //array = &IRArrays[0];
}