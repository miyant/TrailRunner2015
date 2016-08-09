//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################

#define F_CPU   16000000L   //16MHz
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "SerialManager.h"
#include "dynamixel.h"
#include "SensorManager.h"
#include "MotorManager.h"

#include "pid.h"

// ------------------ Defined ------------------
// Action State
#define ACTION_STATE_INIT           2000
#define ACTION_STATE_STOP           2001
#define ACTION_STATE_START          2002
#define ACTION_STATE_MOVE           2003
#define ACTION_STATE_FIXED_MOVE     2004
#define ACTION_STATE_TEST_MODE      3000

// Line Sensor
#define LINE_STATE_BLACK    0
#define LINE_STATE_WHITE    1

// PID Param aaaaa
#define K_P     1.00     // P param
#define K_I     5.00    // I param
#define K_D     0.3  // D param
#define pid_base P_CW_SPEED_NOMAL   // base speed
#define pid_lim 30     // control value
#define DELTA_T 0.002   // delta T

// START SW address
#define SW_START 0x01   // Emergency Stop

// Debug Log ON/OFF
#define DBG 1

#define _LED_ON_

//TEST

// ------------------ Method Definition ------------------
void split( char * s1 );
int initMoveAction(void);
int moveStateAction(void);
int getMoveAction(void);

void getSensors(void);
int getState(void);
void setState(int state);
int getMoveState(void);
void setMoveState(int pre_state, int change_state);

void FixedMotionACtion(void);
void nextMoveAction(int action);

void initEmergencyStop(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

int PID_2(int target_val, int sencer_val);

// ------------------ Global Varriables Definition ------------------

// Serial Message Buf
int serCmd[SERIAL_BUFFER_SIZE] = {0};

// Action State
//int mState = ACTION_STATE_FIXED_MOVE;
//int mState = ACTION_STATE_TEST_MODE;
int mState = ACTION_STATE_INIT;

// Move State
int mMoveState = MOVE_SELECTION_TYPE_STRAIGHT;

// Next Move State
int mBeforeMoveState = MOVE_SELECTION_TYPE_STRAIGHT;

// IR Sensor 
unsigned int IR[ADC_PORT_6 + 1] = {0,0,0,0,0,0,0};

int mMoveCount = 0;

// PID Param
float pGain = 200;   //Proportional Gain
float iGain =  0.2;  //Integral Gain
float dGain =  120;  //Differential Gain
int delay = 10;
int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;   //Previous Error

int diff[2]    = {0,0};
int ret_val[2] = {0,0};
float integral = 0.0;
int target_senser = 0;

// ------------------ Method ------------------

int main(void) {
    
    initEmergencyStop();
    setLED();
    initIRSensor();
    MotorInit();
    initSerial();
    char * readData = NULL; 
    int isFinish = 0;
    
    while(1){
        memset( &serCmd[0], 0x00, sizeof(int) * SERIAL_BUFFER_SIZE );
        if( checkSerialRead() > 0 ) {
            readData = getReadBuffer();
            if( readData != NULL ){
                printf( "readData=%s\n", &readData[0] );
                split( &readData[0] );
                switch( serCmd[0] ) {
                case 1:
                    //MotorControl( 2, serCmd[1] );
                    //setParamMoveAction(serCmd[1], serCmd[2]);
                    break;
                case 999:
                    printf( "finish\n");
                    isFinish = 1;
                    break;
                }
                if( isFinish > 0 ) {
                    //MotorControl( 0, 0 );
                    //setMoveAction(MOVE_SELECTION_TYPE_STOP);
                    break;
                }
                memset( readData, 0x00, SERIAL_BUFFER_SIZE );
            }
            _delay_ms(500);
        } else {
            int state = getState();
            if (~PIND & SW_START) {
                // Emergency stopped
                state = ACTION_STATE_STOP;
            }
            
            switch (state) {
            case ACTION_STATE_INIT:
            	initMoveAction();
                setState(ACTION_STATE_STOP);
                break;
                
            case ACTION_STATE_STOP:
                // motor stop
                setMoveAction(MOVE_SELECTION_TYPE_STOP);
                setState(ACTION_STATE_START);
                break;
                
            case ACTION_STATE_START:
                setState(ACTION_STATE_MOVE);
                break;
                
            case ACTION_STATE_MOVE:
                setState(moveStateAction());
                break;
                
            case ACTION_STATE_FIXED_MOVE:
                FixedMotionACtion();
                break;
                
            case ACTION_STATE_TEST_MODE:
                setMoveAction(MOVE_SELECTION_TYPE_S_MOVE_10);
                break;
                
            default:
                setState(ACTION_STATE_INIT);
                break;
            }
            //_delay_ms(DELAY_MSEC);
            _delay_us(100);
        }
    }
}

void split( char * s1 ) {
    char s2[] = " ,";
    char *tok;
    int cnt = 0;

    tok = strtok( s1, s2 );
    while( tok != NULL ){
//      printf( "%s?n", tok );
        serCmd[cnt++] = atoi(tok);
        tok = strtok( NULL, s2 );  /* 2回目以降 */
    }
}

int initMoveAction(void) {
    setMoveAction(MOVE_SELECTION_TYPE_LEFTTURN);
    _delay_ms(1000);    // 1s
    setMoveAction(MOVE_SELECTION_TYPE_STOP);
}

int moveStateAction(void) {
    int ret_state = ACTION_STATE_MOVE;
    getSensors();

    int pre_move = getMoveState();  // before state
    int next_state;
    int search_count = 0;

    switch (pre_move) {
    case MOVE_SELECTION_TYPE_START:
    case MOVE_SELECTION_TYPE_STOP:
        setMoveAction(pre_move);
        if (pre_move == MOVE_SELECTION_TYPE_START) {
            next_state = MOVE_SELECTION_TYPE_STRAIGHT;
        } else {
            next_state = MOVE_SELECTION_TYPE_START;
            diff[0] = 0;
            diff[1] = 0;
            integral = 0;
        }
        setMoveState(pre_move, next_state);
        break;

    case MOVE_SELECTION_TYPE_STRAIGHT:
    case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
    case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
    case MOVE_SELECTION_TYPE_LEFTSIFT_1:
    case MOVE_SELECTION_TYPE_LEFTSIFT_2:
        PID_2(0, target_senser);
        setParamMoveAction(ret_val[0], ret_val[1]);

        getSensors();
        next_state = getMoveAction();

        setMoveState(pre_move, next_state);
        break;

    case MOVE_SELECTION_TYPE_BACK:
        setMoveAction(pre_move);
        getSensors();
        next_state = getMoveAction();
        
        setMoveState(pre_move, next_state);
        break;
        
    case MOVE_SELECTION_TYPE_RIGHTTURN:
        LED_on(1);
        setMoveAction(pre_move);
        search_count = 0;
        while(1) {
            getSensors();
            next_state = getMoveAction();
            if ( next_state == MOVE_SELECTION_TYPE_STRAIGHT  ||
                 next_state == MOVE_SELECTION_TYPE_RIGHTSIFT_1 ||
                 next_state == MOVE_SELECTION_TYPE_RIGHTSIFT_2 ||
                 next_state == MOVE_SELECTION_TYPE_LEFTSIFT_1 ||
                 next_state == MOVE_SELECTION_TYPE_LEFTSIFT_2) {
                diff[0] = 0;
                diff[1] = 0;
                integral = 0;
                setMoveState(pre_move, next_state);
                break;
            } else {
                //setMoveAction(pre_move);
				setMoveAction(MOVE_SELECTION_TYPE_RIGHTTURN_2);
                setMoveState(pre_move, MOVE_SELECTION_TYPE_RIGHTTURN);
            }
            search_count++;
            if (search_count > 10000) {
                setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
                setMoveState(pre_move, MOVE_SELECTION_TYPE_SEARCH);
                break;
            }
        }
        LED_on(1);
        break;
        
    case MOVE_SELECTION_TYPE_LEFTTURN:
        LED_on(2);
        setMoveAction(pre_move);
        search_count = 0;
        while(1) {
            getSensors();
            next_state = getMoveAction();
            if ( next_state == MOVE_SELECTION_TYPE_STRAIGHT  ||
                 next_state == MOVE_SELECTION_TYPE_LEFTSIFT_1 ||
                 next_state == MOVE_SELECTION_TYPE_LEFTSIFT_2 ||
                 next_state == MOVE_SELECTION_TYPE_RIGHTSIFT_1 ||
                 next_state == MOVE_SELECTION_TYPE_RIGHTSIFT_2) {
                diff[0] = 0;
                diff[1] = 0;
                integral = 0;
                setMoveState(pre_move, next_state);
                break;
            } else {
                //setMoveAction(pre_move);
				setMoveAction(MOVE_SELECTION_TYPE_LEFTTURN_2);
                setMoveState(pre_move, MOVE_SELECTION_TYPE_LEFTTURN);
            }
            search_count++;
            if (search_count > 10000) {
                setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
                setMoveState(pre_move, MOVE_SELECTION_TYPE_SEARCH);
                break;
            }
        }
        LED_on(2);
        break;
        
    case MOVE_SELECTION_TYPE_SEARCH:
        printf("Search move\r\n");
        LED_on(3);
        next_state = getMoveAction();
        if ( next_state != MOVE_SELECTION_TYPE_SEARCH ) {
            // next state
        } else {
            switch (mBeforeMoveState) {
                case MOVE_SELECTION_TYPE_STRAIGHT:
                    next_state = MOVE_SELECTION_TYPE_BACK;
                    break;
                case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
                case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
                    next_state = MOVE_SELECTION_TYPE_RIGHTTURN;
                    break;
                case MOVE_SELECTION_TYPE_LEFTSIFT_1:
                case MOVE_SELECTION_TYPE_LEFTSIFT_2:
                    next_state = MOVE_SELECTION_TYPE_LEFTTURN;
                    break;
                case MOVE_SELECTION_TYPE_RIGHTTURN:
                    next_state = MOVE_SELECTION_TYPE_LEFTTURN;
                    break;
                case MOVE_SELECTION_TYPE_LEFTTURN:
                    next_state = MOVE_SELECTION_TYPE_RIGHTTURN;
                    break;
                case MOVE_SELECTION_TYPE_SEARCH:
                    next_state = MOVE_SELECTION_TYPE_RIGHTTURN;
                    break;
                default:
                    next_state = MOVE_SELECTION_TYPE_LEFTTURN;
                    break;
            }
        }
        if (next_state != MOVE_SELECTION_TYPE_SEARCH) {
            LED_on(3);
        }
        setMoveState(pre_move, next_state);
        break;
        
    case MOVE_SELECTION_TYPE_S_MOVE_1:
        // 1 1 0 0 1 1
        // 1 0 0 0 1 1
        // 1 1 0 0 0 1
        // 1 0 0 0 0 1
        printf("Special Move 1\r\n");
        next_state = getMoveAction();
        setMoveState(pre_move, next_state);
        break;
    case MOVE_SELECTION_TYPE_S_MOVE_2:
        // 0 0 0 0 0 1
        LED_on(4);
        printf("Special Move 2\r\n");
        next_state = getMoveAction();
        setMoveState(pre_move, next_state);
        break;
    case MOVE_SELECTION_TYPE_S_MOVE_3:
        // 1 0 0 0 0 0
        LED_on(5);
        printf("Special Move 3\r\n");
        next_state = getMoveAction();
        setMoveState(pre_move, next_state);
        break;
    case MOVE_SELECTION_TYPE_S_MOVE_4:
        // 0 1 1 1 1 0
        // 0 0 1 1 1 0
        // 0 1 0 1 1 0
        // 0 1 1 0 1 0
        // 0 0 1 0 1 0
        // 0 1 0 0 1 0
        // 0 1 1 1 0 0
        // 0 1 0 1 0 0
        LED_on(6);
        printf( "Special Move 4\r\n" );
        next_state = getMoveAction();
        setMoveState(pre_move, next_state);
        break;
    default:
        setMoveState(0, MOVE_SELECTION_TYPE_STOP);
        ret_state = ACTION_STATE_STOP;
        break;
    }
    
    return ret_state;
}

void getSensors(void) {
    ReadIRSensors(IR);
//  for (int j = ADC_PORT_1; j <= ADC_PORT_6; j++) {
//      printf( "IR[%d] = %d\r\n", j, IR[j] );
//  }
}

int getState(void) {
    return mState;
}

void setState(int state) {
    mState = state;
}

int getMoveState(void) {
    return mMoveState;
}

void setMoveState(int pre_state, int change_state) {
    mBeforeMoveState = pre_state;
    if (pre_state != change_state) {
        mMoveState = change_state;
    }
}

int getMoveAction(void) {
    int ret = 0;
    
    int i_state[ADC_PORT_6 + 1];
    
    for (int i = ADC_PORT_1; i <= ADC_PORT_6; i++) {
        if ( IR[i] >= COMPARE_VALUE ) {
            i_state[i] = LINE_STATE_BLACK;
        } else {
            i_state[i] = LINE_STATE_WHITE;
        }
    }

    // Right 3
    // Center Right_2 6
    // Center Right_1 2
    // Center Left_1  5
    // Center Left_2  1
    // Left  4

    if (i_state[ADC_PORT_3] == LINE_STATE_BLACK) {
        if (i_state[ADC_PORT_4] == LINE_STATE_BLACK) {
            if (i_state[ADC_PORT_6] == LINE_STATE_BLACK) {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 1 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 1 0 1 1 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 1 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 1 0 0 1 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 0 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 1 0 1 0 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 0 1 1
                            ret = MOVE_SELECTION_TYPE_S_MOVE_1;
                        } else {
                            // 1 0 0 0 1 1
                            ret = MOVE_SELECTION_TYPE_S_MOVE_1;
                        }
                    }
                }
            } else {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 1 0 1
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 1 1 0 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 1 0 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 1 0 0 1 0 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 0 0 1
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 1 0 0 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 0 0 1
                            ret = MOVE_SELECTION_TYPE_S_MOVE_1;
                        } else {
                            // 1 0 0 0 0 1
                            ret = MOVE_SELECTION_TYPE_S_MOVE_1;
                        }
                    }
                }
            }
        } else {
            if (i_state[ADC_PORT_6] == LINE_STATE_BLACK) {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 1 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        } else {
                            // 0 0 1 1 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 1 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 0 0 0 1 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 0 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 0 0 1 0 1 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 0 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        } else {
                            // 0 0 0 0 1 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    }
                }
            } else {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 1 0 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        } else {
                            // 0 0 1 1 0 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 1 0 1
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        } else {
                            // 0 0 0 1 0 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 0 0 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        } else {
                            // 0 0 1 0 0 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 0 0 1
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        } else {
                            // 0 0 0 0 0 1
                            ret = MOVE_SELECTION_TYPE_S_MOVE_2;
                        }
                    }
                }
            }
        }
    } else {
        if (i_state[ADC_PORT_4] == LINE_STATE_BLACK) {
            if (i_state[ADC_PORT_6] == LINE_STATE_BLACK) {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 1 1 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 1 1 1 0
                            ret = MOVE_SELECTION_TYPE_RIGHTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 1 1 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 0 1 1 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 0 1 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 1 0 1 0
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 0 1 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 0 0 1 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        }
                    }
                }
            } else {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 1 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 1 1 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 1 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 0 1 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 1 0 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 1 0 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 1 1 0 0 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTTURN;
                        } else {
                            // 1 0 0 0 0 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_3;
                        }
                    }
                }
            }
        } else {
            if (i_state[ADC_PORT_6] == LINE_STATE_BLACK) {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 1 1 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        } else {
                            // 0 0 1 1 1 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 1 1 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        } else {
                            // 0 0 0 1 1 0
                            ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
                            target_senser = 2;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 0 1 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        } else {
                            // 0 0 1 0 1 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 0 1 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        } else {
                            // 0 0 0 0 1 0
                            ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
                            target_senser = 3;
                        }
                    }
                }
            } else {
                if (i_state[ADC_PORT_2] == LINE_STATE_BLACK) {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 1 0 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        } else {
                            // 0 0 1 1 0 0
                            ret = MOVE_SELECTION_TYPE_STRAIGHT;
                            target_senser = 0;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 1 0 0
                            ret = MOVE_SELECTION_TYPE_S_MOVE_4;
                        } else {
                            // 0 0 0 1 0 0
                            ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
                            target_senser = 1;
                        }
                    }
                } else {
                    if (i_state[ADC_PORT_5] == LINE_STATE_BLACK) {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 1 0 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
                            target_senser = -2;
                        } else {
                            // 0 0 1 0 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
                            target_senser = -1;
                        }
                    } else {
                        if (i_state[ADC_PORT_1] == LINE_STATE_BLACK) {
                            // 0 1 0 0 0 0
                            ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
                            target_senser = -3;
                        } else {
                            // 0 0 0 0 0 0
                            ret = MOVE_SELECTION_TYPE_SEARCH;
                        }
                    }
                }
            }
        }
    }
    return ret;
}

void FixedMotionACtion(void) {
    int next_state;
    int timer = 0;
    
    const int count = mMoveCount;
    if (count > MAX_COUNT) {
        return;
    }
    
    switch (count) {
        case 0:
        case 6:
        case 8:
        case 12:
        case 14:
            // Straight to Left Turn
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            nextMoveAction(MOVE_SELECTION_TYPE_LEFTTURN);
            break;
        
        case 1:
        case 7:
        case 9:
        case 13:
        case 15:
            // Left Turn to Straight
            if (DBG) printf("Left Turn %d\r\n", count);
            //TurnMoveLeft();
            setMoveAction(MOVE_SELECTION_TYPE_LEFTTURN);
            nextMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            break;
        
        case 2:
        case 4:
        case 10:
        case 18:
            // Straight to Right Turn
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            nextMoveAction(MOVE_SELECTION_TYPE_RIGHTTURN);
            break;
        
        case 3:
        case 5:
        case 11:
        case 17:
        case 19:
            // Right Turn to Straight
            if (DBG) printf("Right Turn %d\r\n", count);
            //TurnMoveRight();
            setMoveAction(MOVE_SELECTION_TYPE_RIGHTTURN);
            nextMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            break;
        
        case 16:
            // Long Straight to Right Turn
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            while(1) {
                _delay_ms(10);
                getSensors();
                next_state = getMoveAction();
                if ( next_state ==  MOVE_SELECTION_TYPE_RIGHTTURN ) {
                    mMoveCount++;
                    break;
                }
                timer++;
                if (timer > 50000) {
                    // 100s next
                    mMoveCount++;
                    break;
                }
            }
            break;
        
        case 20:
        case 22:
        case 24:
        case 26:
            // Straight to Bifurcate
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            while(1) {
                _delay_ms(10);
                getSensors();
                next_state = getMoveAction();
                if ( next_state ==  MOVE_SELECTION_TYPE_RIGHTTURN ||
                     next_state ==  MOVE_SELECTION_TYPE_LEFTTURN ) {
                    mMoveCount++;
                    break;
                }
                timer++;
                if (timer > 5000) {
                    // 10s next
                    mMoveCount++;
                    break;
                }
            }
            break;
        
        case 21:
        case 23:
        case 25:
        case 27:
            // Bifurcate to Straight
            next_state = getMoveAction();
            if ( next_state ==  MOVE_SELECTION_TYPE_RIGHTTURN ) {
                if (DBG) printf("Right Turn %d\r\n", count);
                //TurnMoveRight();
                setMoveAction(MOVE_SELECTION_TYPE_RIGHTTURN);
            } else if ( next_state ==  MOVE_SELECTION_TYPE_LEFTTURN ) {
                if (DBG) printf("Left Turn %d\r\n", count);
                //TurnMoveLeft();
                setMoveAction(MOVE_SELECTION_TYPE_LEFTTURN);
            }
            nextMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            break;

        case 28:
            // Last Spurt
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            setMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            nextMoveAction(MOVE_SELECTION_TYPE_STOP);
            break;

        case 29:
        default:
            if (DBG) printf("STOP !! %d\r\n", count);
            setMoveAction(MOVE_SELECTION_TYPE_STOP);
            break;
    }
}

void nextMoveAction(int action) {
    int next_state;
    int timeout = 0;
    while(1) {
        _delay_ms(10);
        getSensors();
        next_state = getMoveAction();
        if ( next_state ==  action ) {
            mMoveCount++;
            break;
        }
        timeout++;
        if (timeout > 5000) {
            // 1s next
            mMoveCount++;
            break;
        }
    }
}

struct PID_DATA pidData;

void PID_init(void);
void PID_init(void) {
    pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData); 
}

float PID(float cur_value, float req_value) {
    float pid;
    float error;
    
    error = req_value - cur_value;
    pid = (pGain * error)  + (iGain * eInteg) + (dGain * (error - ePrev));

    eInteg += error;        // integral is simply a summation over time
    ePrev = error;          // save previous for derivative

    return pid;
}

int PID_2(int target_val, int sencer_val) {
    float p,i,d;
    
    diff[0] = diff[1];
    diff[1] = sencer_val - target_val;
    
    integral += ((diff[0] + diff[1]) / 2.0 * DELTA_T);
    
    p = K_P * diff[1];
    i = K_I * integral;
    d = K_D * ((diff[1] - diff[0]) / DELTA_T);
    
    int MV = (int)(pid_lim * (p + i + d));
    
    printf( "MV = %d\r\n", MV );
    
    int right_val = pid_base - MV;
    int left_val = pid_base + MV;
    if (right_val > pid_base + 300) {
        right_val = pid_base + 300;
    } else if (right_val < pid_base - 300) {
        right_val = pid_base - 300;
    }
    if (left_val > pid_base + 300) {
        left_val = pid_base + 300;
    } else if (left_val < pid_base - 300) {
        left_val = pid_base - 300;
    }
    
    ret_val[0] = right_val + 1023;
    ret_val[1] = left_val;
    
    return 0;
}

void DelayMs(uint8_t ms) {
    uint8_t i;
    for( i = 0; i < ms; i++ ) {
        _delay_ms(1);
    }
}

void MainLog(char * msg) {
    if (DBG) {
        printf("Main : %s \r\n", msg);
    }
}

void initEmergencyStop(void) {
    DDRD  = 0x70;
    PORTD = 0x11;
}

void setLED(void) {
#ifdef _LED_ON_
    DDRC  = 0x7F;
    PORTC = 0x7F;
#endif // _LED_ON_
}

void LED_on(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
    PORTC = c^(1<<i);
#endif // _LED_ON_
}

void LED_off(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
    PORTC = ~(c^(1<<i));
#endif // _LED_ON_
}
