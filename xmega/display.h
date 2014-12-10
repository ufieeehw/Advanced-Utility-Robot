/*
 * display.h
 *
 * Created: 12/9/2014 8:43:13 PM
 *  Author: Ian
 */ 


#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "characters.h"

//Utilizes all the pins on the data port, and the upper four pins of the control port

/*
 * enum for each segment
 */
typedef enum {
	SEG1,
	SEG2,
	SEG3,
	SEG4
	}segment;

/*
 * Initialize the display using pins 0-7 on the data port and pin 4-7 on the
 * control port.
 */
void init_display(PORT_t *data, PORT_t *control);

/*
 * Clears all segments of the display
 */
void clear_display(void);

/*
 * Clears passed segment
 */
void clear_seg(segment seg);

/*
 * Sets the active segment to be written/ cleared.
 */
void set_seg(segment seg);

/*
 * Displays a 4 bit number in hex. Default condition is an underscore
 */
void write_num(uint8_t output);

/*
 * Displays S t x x, where the x's make up the 8 bit argument.
 */
void write_state(uint8_t state);

#endif /* DISPLAY_H_ */