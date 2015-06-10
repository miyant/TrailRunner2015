/*
 * CFile1.c
 *
 * Created: 2014/07/03 14:01:39
 *  Author: sapporo62
 */ 

#include "SerialManager.h"

volatile int isRead = 0;
volatile int buffCnt = 0;
char readBuffer[SERIAL_BUFFER_SIZE] = {0};

void initSerial(void){
	printf( "initSerial\n" );
	serial_initialize(57600);
	sei();	// Interrupt Enable
}

void clearSerialBuffer(){
	printf( "clearSerialBuffer\n" );
	isRead = 0;
	memset( &readBuffer[0], 0x00, SERIAL_BUFFER_SIZE );
}

int checkSerialRead(void){
	int ret = 0;
	if( serial_get_qstate() > 0 ){
		unsigned char ReceivedData = getchar();
//		printf( "checkSerialRead %d\n", ReceivedData );
		if( ReceivedData == 10 || ReceivedData == 13 ){
			if( buffCnt > 0 ){
				buffCnt = 0;
				isRead = 1;
				ret = 1;
			}
		}else if( ReceivedData > 0 ){
			readBuffer[buffCnt++] = ReceivedData;
			if( buffCnt >= SERIAL_BUFFER_SIZE-1 ){
				buffCnt = 0;
				isRead = 1;
				ret = 1;
			}
		}
		printf( "readBuffer %s\n", &readBuffer[0] );
	}
	return ret;
}

char * getReadBuffer(){
	printf( "getReadBuffer %d\n", isRead );
	char * ret = NULL;
	if( isRead > 0 ){
		printf( "readBuffer=%s\n", &readBuffer[0] );
		//ret = malloc( sizeof(unsigned char) * BUFFER_SIZE);
		//if( ret != NULL ){
			//printf( "ret != NULL\n" );
			//memcpy( &ret, &readBuffer, sizeof(unsigned char) * BUFFER_SIZE );
			//memset( &readBuffer[0], 0x00, BUFFER_SIZE );
			//isRead = 0;
			//printf( "ret=%s\n", ret );
		//}
		ret = &readBuffer[0];
		isRead = 0;
	}
	return ret;	
}
