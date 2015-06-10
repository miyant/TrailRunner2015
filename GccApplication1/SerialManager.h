/*
 * IncFile1.h
 *
 * Created: 2014/07/03 14:01:02
 *  Author: sapporo62
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "serial.h"

#define SERIAL_BUFFER_SIZE 32

extern void initSerial(void);
extern void clearSerialBuffer();
extern int checkSerialRead(void);
extern char * getReadBuffer();

#endif /* INCFILE1_H_ */