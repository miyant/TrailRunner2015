/*
 * IRSensor.h
 *
 * Created: 2014/08/08
 *  Author: sapporo62
 */ 

#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include <stdio.h>
#include <avr/io.h>

#define		COMPARE_VALUE	450

#define		ADC_PORT_1	1
#define		ADC_PORT_2	2
#define		ADC_PORT_3	3
#define		ADC_PORT_4	4
#define		ADC_PORT_5	5
#define		ADC_PORT_6	6

extern void initIRSensor();
extern unsigned int ReadIRSensor();

#endif IRSENSOR_H_
