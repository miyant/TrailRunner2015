/*
 * SensorManager.h
 *
 * Created: 2014/08/19 18:57:13
 *  Author: Administrator
 */ 

#ifndef SENSORMANAGER_H_
#define SENSORMANAGER_H_

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/common.h>

#define     ADC_PORT_1  1
#define     ADC_PORT_2  2
#define     ADC_PORT_3  3
#define     ADC_PORT_4  4
#define     ADC_PORT_5  5
#define     ADC_PORT_6  6

//#define COMPARE_VALUE 450//450
#define COMPARE_VALUE 300//450

unsigned int values;

void initIRSensor(void);
unsigned int ReadIRSensor(unsigned int ch);
void ReadIRSensors(unsigned int * sensors);
void getIRSensors(unsigned int * array);

#endif /* SENSORMANAGER_H_ */
