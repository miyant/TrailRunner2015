/*
 * MotorManager.c
 *
 * Created: 2014/10/29 14:46:23
 *  Author: Administrator
 */ 
#include "MotorManager.h"

#define DBG 1
//#define _MOTOR_OFF_

void TurnMoveRight(void);
void TurnMoveLeft(void);
void TurnLowMoveRight(void);
void TurnLowMoveLeft(void);
void StraightMove(void);
void StraightMoveRightSift(void);
void StraightMoveLeftSift(void);
void StraightMoveRightSift2(void);
void StraightMoveLeftSift2(void);
void StopMove(void);
void BackMove(void);
void TestMode(void);

void MotorInit(void) {
    dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
    //Wheel Mode
    dxl_write_word( RIGHT_MOTOR, P_CW_ANGLE_LIMIT_L, 0 );
    dxl_write_word( RIGHT_MOTOR, P_CW_ANGLE_LIMIT_H, 0 );
    dxl_write_word( RIGHT_MOTOR, P_CCW_ANGLE_LIMIT_L, 0 );
    dxl_write_word( RIGHT_MOTOR, P_CCW_ANGLE_LIMIT_H, 0 );
    dxl_write_word( LEFT_MOTOR, P_CW_ANGLE_LIMIT_L, 0 );
    dxl_write_word( LEFT_MOTOR, P_CW_ANGLE_LIMIT_H, 0 );
    dxl_write_word( LEFT_MOTOR, P_CCW_ANGLE_LIMIT_L, 0 );
    dxl_write_word( LEFT_MOTOR, P_CCW_ANGLE_LIMIT_H, 0 );
    //Set Torque
    dxl_write_word( RIGHT_MOTOR, 24, 1 );
    dxl_write_word( LEFT_MOTOR,  24, 1 );
    //Set EEP Lock
    dxl_write_word( RIGHT_MOTOR, P_EEP_LOCK, 1 );
    dxl_write_word( LEFT_MOTOR, P_EEP_LOCK, 1 );
    // Set goal speed
    dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
    _delay_ms(1000);
}

void MotorControl(int id, int power) {
#ifndef _MOTOR_OFF_
    int CommStatus = COMM_RXSUCCESS;
//  printf( "%d %d\n", id, power );
    dxl_write_word( id, P_GOAL_SPEED_L, power );
    CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
        PrintErrorCode();
    else
        PrintCommStatus(CommStatus);
#endif // _MOTOR_OFF_
}

void setMoveAction(int type) {
    switch (type) {
        case MOVE_SELECTION_TYPE_START:
        case MOVE_SELECTION_TYPE_STRAIGHT:
            printf("Straight Move\r\n");
            StraightMove();
            break;
        case MOVE_SELECTION_TYPE_STOP:
            printf("Stop Move\r\n");
			StopMove();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
            printf("Right Sift 1\r\n");
            StraightMoveRightSift();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_1:
            printf("Left Sift 1\r\n");
            StraightMoveLeftSift();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
            printf("Right Sift 2\r\n");
            StraightMoveRightSift2();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_2:
            printf("Left Sift 2\r\n");
            StraightMoveLeftSift2();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN:
            printf("Right Turn\r\n");
            //TurnMoveRight();
            TurnLowMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN:
            printf("Left Turn\r\n");
            //TurnMoveLeft();
            TurnLowMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN_2:
            printf("Right Turn 2\r\n");
			TurnMoveRight();
            //TurnLowMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN_2:
            printf("Left Turn 2\r\n");
			TurnMoveLeft();
            //TurnLowMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_BACK:
            printf("Back Move\r\n");
            StopMove();
            _delay_ms(10);
            BackMove();
            _delay_ms(1000);
            break;
        case MOVE_SELECTION_TYPE_S_MOVE_10:
            TestMode();
        default:
            break;
    }
}

void setParamMoveAction(int right, int left) {
    MotorControl( RIGHT_MOTOR, right );
    MotorControl( LEFT_MOTOR, left );
}

void StopMove(void) {
    MotorControl( RIGHT_MOTOR, 1024 );
    MotorControl( LEFT_MOTOR, 0 );
}

void StraightMove(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_NOMAL );
    MotorControl( LEFT_MOTOR, P_CW_SPEED_NOMAL );
}

void StraightMoveRightSift(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_TURN );
    MotorControl( LEFT_MOTOR, P_CW_SPEED_NOMAL );
}

void StraightMoveLeftSift(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_NOMAL );
    MotorControl( LEFT_MOTOR, P_CW_SPEED_TURN );
}

void StraightMoveRightSift2(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_TURN_2 );
    MotorControl( LEFT_MOTOR, P_CW_SPEED_NOMAL );
}

void StraightMoveLeftSift2(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_NOMAL );
    MotorControl( LEFT_MOTOR, P_CW_SPEED_TURN_2 );
}

void TurnMoveRight(void) {
    MotorControl( RIGHT_MOTOR, P_CW_SPEED_TURN );
    //  MotorControl( RIGHT_MOTOR, 1024 );
    //  MotorControl( RIGHT_MOTOR, P_CW_SPEED_NOMAL );

    MotorControl( LEFT_MOTOR, P_CW_SPEED_TURN );
    //  MotorControl( LEFT_MOTOR, P_CW_SPEED_NOMAL );
}

void TurnMoveLeft(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_TURN );
    //  MotorControl( RIGHT_MOTOR, P_CCW_SPEED_NOMAL );

    MotorControl( LEFT_MOTOR, P_CCW_SPEED_TURN );
    //  MotorControl( LEFT_MOTOR, 0 );
    //  MotorControl( LEFT_MOTOR, P_CCW_SPEED_NOMAL );
}

void TurnLowMoveRight(void) {
    MotorControl( RIGHT_MOTOR, P_CW_SPEED_TURN_2 );
    MotorControl( LEFT_MOTOR, P_CW_SPEED_TURN_2 );
}

void TurnLowMoveLeft(void) {
    MotorControl( RIGHT_MOTOR, P_CCW_SPEED_TURN_2 );
    MotorControl( LEFT_MOTOR, P_CCW_SPEED_TURN_2 );
}

void BackMove(void) {
    MotorControl( RIGHT_MOTOR, P_CW_SPEED_TURN );
    MotorControl( LEFT_MOTOR, P_CCW_SPEED_TURN );
}

void TestMode(void) {
    long move_time = 100000;
    long stop_time = 50000;
    StopMove();
    _delay_ms(stop_time);

    StraightMove();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveLeftSift();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveRightSift();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveLeftSift2();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveRightSift2();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    TurnMoveRight();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    TurnMoveLeft();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    BackMove();
    _delay_ms(move_time);
}

void PrintErrorCode() {
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    printf("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    printf("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    printf("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    printf("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    printf("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    printf("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    printf("Instruction code error!\n");
}

// Print communication result
void PrintCommStatus(int CommStatus) {
    switch(CommStatus)  {
        case COMM_TXFAIL:
            printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
            break;

        case COMM_TXERROR:
            printf("COMM_TXERROR: Incorrect instruction packet!\n");
            break;

        case COMM_RXFAIL:
            printf("COMM_RXFAIL: Failed get status packet from device!\n");
            break;

        case COMM_RXWAITING:
            printf("COMM_RXWAITING: Now recieving status packet!\n");
            break;

        case COMM_RXTIMEOUT:
            printf("COMM_RXTIMEOUT: There is no status packet!\n");
            break;

        case COMM_RXCORRUPT:
            printf("COMM_RXCORRUPT: Incorrect status packet!\n");
            break;

        default:
            printf("This is unknown error code!\n");
            break;
    }
}
