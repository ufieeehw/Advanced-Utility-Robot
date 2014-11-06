#include <asf.h>
#include <avr/interrupt.h>
#include "MB7060_driver.h"

#define cmtoADC(distance) ((uint16_t)((distance) * .0049 / 0.002441406))
#define tenMiliseconds 25000
#define triggerPeriod 3120
#define samplePeriod 3125

volatile uint16_t left_sonar_value = 0;
volatile uint16_t middle_sonar_value = 0;
volatile uint16_t right_sonar_value = 0;

void ADC_INIT(void){
	ADCA_CTRLA |= ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm |ADC_ENABLE_bm;
	ADCA_CTRLB |= ADC_CONMODE_bm |ADC_FREERUN_bm | ADC_RESOLUTION_12BIT_gc;
	ADCA_REFCTRL |= ADC_REFSEL_INTVCC_gc;
	ADCA_EVCTRL |= ADC_SWEEP_012_gc | ADC_EVACT_SWEEP_gc;
	ADCA_PRESCALER |= ADC_PRESCALER_DIV32_gc;
	ADCA_CH0_CTRL |= 0x81; // Start scan CH0 w/ single ended input mode
	ADCA_CH1_CTRL |= 0x81;
	ADCA_CH2_CTRL |= 0x81;
	ADCA_CH1_MUXCTRL |= ADC_CH_MUXPOS0_bm;
	ADCA_CH2_MUXCTRL |= ADC_CH_MUXPOS1_bm;
	PORTA_DIRCLR |= 0x07; // Set pin 0 on PORTA to input
}

void MB7060_INIT(void){
	ADC_INIT();

	// Set up a pin to trigger the sonar
	PORTD.DIRSET = 0x80;
	PORTD.OUTCLR = 0x80;

	// 10ms timer
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
}

int main (void)
{
	MB7060_INIT();
	
	PORTD.DIRSET = 0x03;
	PORTD.OUTSET = 0x03;
	
	while(1){
		
	}
}

ISR(TCC0_CCA_vect)
{
	// disable sonar
	PORTD.OUTCLR = 0x80;
	PORTD.OUTTGL = 0x02;
		
	// take 7 samples
	if(TCC0.CCA < (samplePeriod * 7)){
		TCC0.CCA += samplePeriod;	
	}
	
	left_sonar_value += ADCA.CH0.RES;
	middle_sonar_value += ADCA.CH1.RES;
	right_sonar_value += ADCA.CH2.RES;
}

ISR(TCC0_CCB_vect)
{
	// Trigger sonar
	PORTD.OUTSET = 0x80;
	TCC0.CCB += samplePeriod;
}


ISR(TCC0_OVF_vect) {
	TCC0.CCA = samplePeriod;
	TCC0.CCB = triggerPeriod;
	
	// disable sonar
	PORTD.OUTCLR = 0x80;
	PORTD.OUTTGL = 0x03;
	
	// Get the 8th sample and average
	left_sonar_value = (left_sonar_value + ADCA.CH0.RES) >> 3;
	middle_sonar_value = (middle_sonar_value + ADCA.CH1.RES) >> 3;
	right_sonar_value = (right_sonar_value + ADCA.CH2.RES) >> 3;
	
	
	left_sonar_value = 0;
	middle_sonar_value = 0;
	right_sonar_value = 0;
}