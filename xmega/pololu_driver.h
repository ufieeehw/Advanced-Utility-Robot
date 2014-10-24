/*
 * pololu_driver.h
 *
 * Created: 10/15/2014 5:45:04 PM
 *  Author: Austin
 */

#ifndef POLOLU_DRIVER_H_INCLUDED
#define POLOLU_DRIVER_H_INCLUDED

#include <asf.h>

typedef struct {
	PORT_t *PORT;
	TC0_t * TC0;
	TC1_t* TC1;
	} pololu_t;

extern pololu_t pololu_left;
extern pololu_t pololu_right;

void pololuInit(void);
void pololuDrive(pololu_t *pololu, int8_t percent);

#endif // POLOLU_DRIVER_H_