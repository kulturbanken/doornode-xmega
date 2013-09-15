#include <stdio.h>
#include <stddef.h> 
#include <avr/io.h>
#include <avr/pgmspace.h> 

#include "adc.h"

static uint8_t read_calibration_byte(uint8_t index)
{ 
	uint8_t result; 

	/* Load the NVM Command register to read the calibration row. */ 
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */ 
	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

	return( result ); 
}

void adc_init()
{
	ADCA.CALL = read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0)); 
	ADCA.CALH = read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));

	ADCA.CTRLA = 0x01; /* enable ADC */
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm; /* 12bit + signed mode */
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc /* | ADC_TEMPREF_bm*/;
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
}

#if 0
int16_t adc_get_cpu_temp()
{
	int16_t ref, degrees_x10;
	float kelvin_per_adc_x10;

	ref =  read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE0));
	ref += read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE1)) << 8;
	kelvin_per_adc_x10 = ((273 + 85)*10) / (float)ref; // reference is ADC reading at 85C, scaled by 10 to get units of 0.1C

	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc | ADC_CH_GAIN_2X_gc;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXINT_TEMP_gc; /* Temperature */
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
	while(!ADCA.CH0.INTFLAGS);

	degress_x10 = ADCA.CH0RES;	
	degrees_x10 *= kelvin_per_adc_x10;
	degrees_x10 -= 2730;

	return degrees_x10;
}
#endif

int16_t adc_read(uint8_t channel)
{
	int16_t adval;

	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_2X_gc;
	ADCA.CH0.MUXCTRL = channel << 3;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
	while(!ADCA.CH0.INTFLAGS);
	adval = ADCA.CH0RES;
	//if (adval < 0)
	//	return 0;
	return adval;
}
