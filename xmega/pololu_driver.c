/**
 * \file
 *
 * \brief Empty user application template
 *
 */

#include "pololu_driver.h"
#include "qdec_driver.h"

#define PERIOD 50
#define MAGNITUDE (PERIOD/2)
#define FORWARD 1
#define REVERSE 0

pololu_t pololu_left = {
		.PORT = &PORTE,
		.TC0 = &TCE0,
		.TC1 = &TCE1
	};
	
pololu_t pololu_right = {
		.PORT = &PORTE,
		.TC0 = &TCC0,
		.TC1 = &TCC1
	};

void pololuInit(void){
	// LEFT MOTOR
	PORTE.DIR = 0xCF;							// Set PORT directions
	PORTE.OUT = 0x0E;							// Set inital outputs
		
	TCE0.CTRLA |= TC_CLKSEL_DIV1_gc;				// Set clock divider to 64
	TCE0.CTRLB |= 0x10 | TC_WGMODE_SINGLESLOPE_gc;  // Enable single slope PWM. Enable OC A.
	TCE0.PER = PERIOD;    							// PER = (40kHz*2MHz)/1 = 50
	TCE0.CCA = MAGNITUDE;
	
	// RIGHT MOTOR
	PORTC.DIR = 0xCF;							// Set PORT directions
	PORTC.OUT = 0x0E;							// Set inital outputs
		
	TCC0.CTRLA |= TC_CLKSEL_DIV1_gc;				// Set clock divider to 64
	TCC0.CTRLB |= 0x10 | TC_WGMODE_SINGLESLOPE_gc;  // Enable single slope PWM. Enable OC A.
	TCC0.PER = PERIOD;    							// PER = (40kHz*2MHz)/1 = 50
	TCC0.CCA = MAGNITUDE;
}

 void pololuDrive(pololu_t *pololu, int8_t percent){
	// TODO
	// Should probably write some respectible code
	pololu->TC0->CCA = (MAGNITUDE + (MAGNITUDE * ((double)percent/100)));
}

// int main (void)
// {	
// 	pololuInit();
// 	pololuDrive(&pololu_left, 50);
// 	while(1){
// 		asm("nop");
// 	}
// }
