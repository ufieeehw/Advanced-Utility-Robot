/*
 * towbot_motor_controller.h
 *
 * Created: 10/15/2014 10:20:35 PM
 *  Author: Austin
 */ 

#ifndef TOWBOT_MOTOR_CONTROLLER_H_INCLUDED
#define TOWBOT_MOTOR_CONTROLLER_H_INCLUDED

#include "message.h"

#define Y_CALIBRATION 0x03
#define X_CALIBRATION 0x00
#define MAX_VALUE 0x86

void towBot_Init(void);
int towbot_msg(Message m);

#endif // TOWBOT_MOTOR_CONTROLLER_H_
