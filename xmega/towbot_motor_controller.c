/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <math.h>
#include "towbot_motor_controller.h"
#include "pololu_driver.h"
#include "nunchuk_driver.h"
#include "message.h"
#include "types.h"


uint8_t data[DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int8_t xpos = 0;
int8_t ypos = 0;
uint8_t mag = 0;
int8_t percent_mag = 0;
int8_t percent_left = 0;
int8_t percent_right = 0;

void towBot_Init(){
	PORTQ.DIRSET = 0xF;
	PORTQ.OUT = 0x0;
	
	pololuInit();
	
	//redundant
	pololuDrive(&pololu_left, 0);
	pololuDrive(&pololu_right, 0);
}

int towbot_msg(Message m){
	pololuDrive(&pololu_left, m.data[0]);
	pololuDrive(&pololu_right, m.data[1]);
}