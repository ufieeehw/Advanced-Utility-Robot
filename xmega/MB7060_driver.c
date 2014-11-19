#include <asf.h>
#include <avr/interrupt.h>
#include "MB7060_driver.h"
#include "message.h"
#include "types.h"

/*
 * Macro to make distance thresholds easier. All actual calculations and compares
 *  will stay was uint16_t to improve performance.
 */
#define cmtoADC(distance) ((uint16_t)((distance) * .0049 / 0.002441406))

// All periods assume F_CPU = 32MHz
#define tenHertz 50000	// 10mS
#define sixteenMiliseconds 8000	// 16mS
#define triggerPeriod 1			//
#define samplePeriod 32500		// 65mS

// Buffers to hold averaged sonar values
volatile static uint16_t left_sonar_value = 0;
volatile static uint16_t middle_sonar_value = 0;
volatile static uint16_t right_sonar_value = 0;

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
 *  They will also require a single pin which goes high for 20uS(CCA) to trigger each sonar
 *  sample. The sonar samples will be stored in the xxx_sonar_value buffer, respective to
 *  each sonar. The sonar information is updated at 81mS. Between 65mS and 81mS, eight (8)
 *  samples are taken (CCB) and at the end of the 81mS they are averaged together
 *  and output. This entire process is interrupt driven.
 */
void MB7060_INIT(void){
	ADC_INIT();

	// Set up a pin to trigger the sonar
	PORTD.DIRSET = 0x80;
	PORTD.OUTCLR = 0x80;

	/*
	 * 10ms timer using Timer/Counter C0. CCA is used to trigger the sonars while CCB is used
	 *   to capture them.
	 */
	TCC0.CTRLA = TC_CLKSEL_OFF_gc;
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCC0.INTCTRLA = TC_OVFINTLVL_MED_gc;
	TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc | TC_CCBINTLVL_MED_gc;
	TCC0.CCA = triggerPeriod;
	TCC0.CCB = samplePeriod;
	TCC0.PER = tenHertz;
	TCC0.CTRLA = TC_CLKSEL_DIV64_gc;
	
	PMIC.CTRL |= PMIC_MEDLVLEN_bm; //Enable medium level interrupts for receiver
	sei();  //Enable global interrupts
	
	//TESTING CODE
	PORTD.DIRSET = 0x03;
	PORTD.OUTSET = 0x03;
}


/*
 * ISR to trigger sonar. Pin is pulled high here, and ~20uS later pulled
 *  low.
 */
ISR(TCC0_CCA_vect)
{
	// Trigger sonar
	if (TCC0.CCA == triggerPeriod)
	{
		PORTD.OUTSET = 0x80;
		TCC0.CCA += 10;
	} else { // stop sonar trigger and reset
		PORTD.OUTCLR = 0x80;
		TCC0.CCA = triggerPeriod;
	}
}


/*
 * ISR to handle collection of sonar data. Will be triggered 7 times before the
 *  final (8th) data point is collected and averaged. This acts as a low-pass
 *  filter for the sonar ADCs.
 */
ISR(TCC0_CCB_vect)
{
	// Disable sonar
	PORTD.OUTCLR = 0x80;
	
	// If just starting to sample, clear buffers
	if (TCC0.CCB == samplePeriod)
	{
		left_sonar_value = 0;
		middle_sonar_value = 0;
		right_sonar_value = 0;		
	}
	
	// take 7 samples
	if(TCC0.CCB < ((uint16_t)samplePeriod + sixteenMiliseconds)){
		TCC0.CCB += 1000;
		left_sonar_value += ADCA.CH0.RES;
		middle_sonar_value += ADCA.CH1.RES;
		right_sonar_value += ADCA.CH2.RES;	
	} else { // average the samples together and reset counter compare
		TCC0.CCB = samplePeriod;
		left_sonar_value = (left_sonar_value >> 3);
		middle_sonar_value = (middle_sonar_value >> 3);
		right_sonar_value = (right_sonar_value >> 3);
	}
}

/*
 * Overflow ISR every 100ms.
 */
ISR(TCC0_OVF_vect) {
	PORTD.OUTTGL = 0x01;
	// Send sonar data to ROS
	Message out = get_msg(SONAR_DATA_TYPE, 6);
	*out.data = left_sonar_value;
	*(out.data + 2) = middle_sonar_value;
	*(out.data + 4) = right_sonar_value;
	queue_push(out,OUT_QUEUE);  //send ack
}