#include <asf.h>
#include <avr/interrupt.h>
#include "MB7060_driver.h"

// Macro to make distance thresholds easier. All actual calculations and compares
//  will stay was uint16_t to improve performance.
#define cmtoADC(distance) ((uint16_t)((distance) * .0049 / 0.002441406))

// All periods assume F_CPU = 32MHz
#define tenMiliseconds 50000 
#define triggerPeriod 6240
#define samplePeriod 6250

// Buffers to hold averaged sonar values
volatile uint16_t left_sonar_value = 0;
volatile uint16_t middle_sonar_value = 0;
volatile uint16_t right_sonar_value = 0;

/*
 * ADCA channels 0, 1, and 2 are set up in single ended, signed, 12bit right aligned mode
 *  with freerun sampling. 12bit resolution gives the accuracy, and at 32MHz is plenty fast.
*/
void ADC_INIT(void){
	ADCA_CTRLA |= ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm |ADC_ENABLE_bm;
	ADCA_CTRLB |= ADC_CONMODE_bm |ADC_FREERUN_bm | ADC_RESOLUTION_12BIT_gc;
	ADCA_REFCTRL |= ADC_REFSEL_INTVCC_gc;
	ADCA_EVCTRL |= ADC_SWEEP_012_gc | ADC_EVACT_SWEEP_gc;
	ADCA_PRESCALER |= ADC_PRESCALER_DIV32_gc;
	ADCA_CH0_CTRL |= 0x81; // Start scan CH0 w/ single ended input mode
	ADCA_CH1_CTRL |= 0x81; // Start scan CH1 w/ single ended input mode
	ADCA_CH2_CTRL |= 0x81; // Start scan CH2 w/ single ended input mode
	ADCA_CH1_MUXCTRL |= ADC_CH_MUXPOS0_bm;
	ADCA_CH2_MUXCTRL |= ADC_CH_MUXPOS1_bm;
	PORTA_DIRCLR |= 0x07; // Set pin 0 on PORTA to input
}


/*
 * The MB7060 sonars will require three (3) ADC channels to get distance information.
 *  They will also require a single pin which goes high for 20uS(CCB) to trigger each sonar
 *  sample. The sonar samples will be stored in the xxx_sonar_value buffer, respective to
 *  each sonar. The sonar information is updated at 10Hz. During that 10Hz, eight (8)
 *  samples are taken (CCA) and at the end of the 100ms period, they are averaged together
 *  and output. This entire process is interrupt driven.
*/
void MB7060_INIT(void){
	ADC_INIT();

	// Set up a pin to trigger the sonar
	PORTD.DIRSET = 0x80;
	PORTD.OUTCLR = 0x80;

	/*
	 * 10ms timer using Timer/Counter C0. CCB is used to trigger the sonars while CCA is used
	 *   to capture them. It is important to note that when the sonars are triggered it is for
	 *   next capture. In other words, if I trigger the sonar and then follow with a capture,
	 *   that capture is NOT the information from the trigger I just sent. The speed of sound
	 *   requires that there be a few milliseconds between when a sonar ping is sent and when it
	 *   is received. This method triggers the next round of information while collecting the
	 *   previous round. This is fine, because of the speed of movement is relatively slow and
	 *  the sample rate is plenty fast.
	*/
	TCC0.CTRLA = TC_CLKSEL_OFF_gc;
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCC0.INTCTRLA = TC_OVFINTLVL_MED_gc;
	TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc | TC_CCBINTLVL_MED_gc;
	TCC0.CCA = samplePeriod;
	TCC0.CCB = triggerPeriod;
	TCC0.PER = tenMiliseconds;
	TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
	
	PMIC.CTRL |= PMIC_MEDLVLEN_bm; //Enable medium level interrupts for receiver
	sei();  //Enable global interrupts
	
	//TESTING CODE
	PORTD.DIRSET = 0x03;
	PORTD.OUTSET = 0x03;
}

/*
 * ISR to handle collection of sonar data. Will be triggered 7 times before the
 *  final (8th) data point is collected and averaged at 100ms.
*/
ISR(TCC0_CCA_vect)
{
	// Disable sonar
	PORTD.OUTCLR = 0x80;
	PORTD.OUTTGL = 0x02;  // TEST
		
	// take 7 samples
	if(TCC0.CCA < (samplePeriod * 7)){
		TCC0.CCA += samplePeriod;	
	}
	
	left_sonar_value += ADCA.CH0.RES;
	middle_sonar_value += ADCA.CH1.RES;
	right_sonar_value += ADCA.CH2.RES;
}

/*
 * ISR to trigger sonar. Pin is pulled high here, and ~20uS later pulled
 *  low in CCB ISR. Offset each time by samplePeriod.
*/
ISR(TCC0_CCB_vect)
{
	// Trigger sonar
	PORTD.OUTSET = 0x80;
	TCC0.CCB += samplePeriod;
}

/*
 * Overflow ISR every 100ms. Handles the 8th and final capture, averages the
 *  data and outputs it for use. This ISR is also responsible for resetting
 *  everything for the next round of sampling.
*/
ISR(TCC0_OVF_vect) {
	TCC0.CCA = samplePeriod;
	TCC0.CCB = triggerPeriod;
	
	// Disable sonar
	PORTD.OUTCLR = 0x80;
	PORTD.OUTTGL = 0x03; // TEST
	
	// Get the 8th sample and average
	left_sonar_value = (left_sonar_value + ADCA.CH0.RES) >> 3;
	middle_sonar_value = (middle_sonar_value + ADCA.CH1.RES) >> 3;
	right_sonar_value = (right_sonar_value + ADCA.CH2.RES) >> 3;
	
	// TODO
	// Output Data
	
	// Clear buffer values
	left_sonar_value = 0;
	middle_sonar_value = 0;
	right_sonar_value = 0;
}