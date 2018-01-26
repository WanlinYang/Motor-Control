#include "isense.h"
#include <xc.h>

unsigned int adc_sample_convert(int pin){
	unsigned int elapsed = 0, finish_time = 0;
	AD1CHSbits.CH0SA = pin;
	AD1CON1bits.SAMP = 1;
	elapsed = _CP0_GET_COUNT();
	finish_time = elapsed + SAMPLE_TIME;
	while(_CP0_GET_COUNT()<finish_time){;}  // sample for more than 250 ns
	AD1CON1bits.SAMP = 0;
	while(!AD1CON1bits.DONE){;}
	return ADC1BUF0;
}