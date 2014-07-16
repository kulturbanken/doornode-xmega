#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "adc.h"
#include "serial.h"
#include "iocard.h"
#include "timer.h"
#include "i2c.h"
#include "wdt_driver.h"

#define IOCARD_FIRMWARE_VERSION 2
#define DEBUG_PRINT_TIMER  8

//#define SERIAL_DEBUG_PRINT

static uint8_t dip_switch;
static uint8_t i2c_address;
static uint8_t over_current = 0;
static bool current_control[8];

volatile uint8_t digital_out = 0;
volatile iocard_data_t iocard_data;

#define MED_VALUES 5   // samples for median calculation, must be an odd number
#define AVG_VALUES 4   // values saved for average calculation

static struct {
	uint16_t samples[MED_VALUES];
	uint16_t values[AVG_VALUES];
	int samples_offset;
	int values_offset;
} ad_array[12];

static void add_ad_sample(int channel, uint16_t sample)
{
	int samples_offset = (ad_array[channel].samples_offset + 1) % MED_VALUES;
	ad_array[channel].samples_offset = samples_offset;

	ad_array[channel].samples[samples_offset] = sample;

	/* Now sort to find median */
	uint16_t sorted[MED_VALUES];
	int j;
	for (j = 0; j < MED_VALUES; j++)
		sorted[j] = ad_array[channel].samples[j];

	/* Only sort half of the values, since we just need the one in the middle */
	for (j = 0; j < (MED_VALUES >> 1) + 1; j++) {
		int k, min = j;
		for (k = j + 1; k < MED_VALUES; k++)
			if (sorted[k] < sorted[min])
				min = k;
		const uint16_t temp = sorted[j];
		sorted[j] = sorted[min];
		sorted[min] = temp;
	}

	uint16_t median = sorted[(MED_VALUES >> 1) + 1];

	/* Insert the newly calculated median value in list of values */
	int values_offset = (ad_array[channel].values_offset + 1) % AVG_VALUES;
	ad_array[channel].values_offset = values_offset;
	ad_array[channel].values[values_offset] = median;
}

static uint16_t get_average_ad_value(int channel)
{
	int j;
	uint32_t total = 0;
	for (j = 0; j < AVG_VALUES; j++)
		total += ad_array[channel].values[j];

	return (total / AVG_VALUES);
}

/*
	Since the ADC is nothing but linear, the multiplication factor differ depending 
	on measure ranges.

	After some investigations, 1.45 seems to be some sort of happy medium.

*/
static int ad_to_ma(uint16_t ad)
{
	return (int)(ad * 1.45);
}

static void update_digital_output(void)
{
	/* Set all requested output pins except those shutdown
	   due to over current */
	uint8_t out_byte = digital_out & (~over_current);

	/* 3 LSB is on PORTE, rest on PORTC */
	PORTE.OUT = (PORTE.OUT & (~0x07)) | (out_byte & (0x07));
	PORTC.OUT = (out_byte & (~0x07)) | (PORTC.OUT & 0x07);
}

static inline uint8_t get_digital_in()
{
	uint8_t ret = 0;

	/* PR0 + PR1 on bit 0 and 1 respectively */
	ret |= PORTR.IN & 0x03;

	/* PE3 on bit 3 */
	ret |= PORTE.IN & (1<<3);

	return ret;
}

static void update_iocard_struct()
{
	uint8_t n;
	
	cli();
	iocard_data.version = IOCARD_FIRMWARE_VERSION;
	iocard_data.dip_switch = dip_switch;
	iocard_data.digital_in = get_digital_in();
	iocard_data.digital_out = digital_out;
	iocard_data.over_current = over_current;
	for (n = 0; n < 12; n++)
		iocard_data.analog_in[n] = get_average_ad_value(n);
	iocard_data.int_temp = 0;
	iocard_data.int_voltage = 0;
	iocard_data.int_bandgap = 0;
	sei();
}

static unsigned int laps = 0;

#ifdef SERIAL_DEBUG_PRINT
static void debug_print()
{
	int n;
	static int flipflop = 0;

	if (flipflop == 1) {
		flipflop = 0;
	} else {
		serprintf("\r");
		for (n = 0; n < 12; n++) {
			serprintf("%5d | ", n < 8 ? ad_to_ma(get_average_ad_value(n)) : get_average_ad_value(n));
		}
		serprintf("%5d", laps);
#if 0
		avg = get_average_ad_value(2);
		if (avg < low)
			low = avg;
		if (avg > high)
			high = avg;
		serprintf("CH2 = %-5d low = %-5d high = %-5d diff = %-3d | PORTD = 0x%02x laps = %d Din = 0x%02x mA = %d digital_out = 0x%02x\r\n",
			  avg, low, high, high - low, PORTD.IN, laps, get_digital_in(), ad_to_ma(avg), digital_out);
#endif
		flipflop = 1;
		laps = 0;
	}
}
#endif

int main(void)
{
	//set_32mhz();

	PORTA.DIR = 0x00;
	PORTB.DIR = 0x00;
	PORTC.DIR = 0b11111100;
	PORTC.OUT = 0x00;
	PORTD.DIR = 0x00;
	PORTCFG.MPCMASK = 0xFF; /* "select" all pins */
	PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm; /* enable pull-up and inverted input on selected pins */
	PORTE.DIR = 0b00000111;
	PORTE.OUT = 0x00;
	PORTR.DIR = 0x00;
	/* Invert digital input */
	PORTE.PIN3CTRL = PORT_INVEN_bm;
	PORTR.PIN0CTRL = PORT_INVEN_bm;
	PORTR.PIN1CTRL = PORT_INVEN_bm;
	_delay_ms(1);

	dip_switch = PORTD.IN;
	i2c_address = dip_switch >> 6;

	/* Reset DIP pins for serial use etc */
	//PORTCFG.MPCMASK = 0xFF;
	//PORTD.PIN0CTRL = 0;

#ifndef SERIAL_DEBUG_PRINT
	/* Enable 8 second watchdog, will only reset on I2C requests */
	WDT_EnableAndSetTimeout( WDT_PER_8KCLK_gc );
#endif

#ifdef SERIAL_DEBUG_PRINT
	serial_init(0);
#endif
	adc_init();
	timer_init();
	i2c_init(i2c_address);

	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; 
	sei();

	_delay_ms(100);

	int n;

#ifdef SERIAL_DEBUG_PRINT
	serprintf("\r\nStarting up...\r\n");
	serprintf("DIP switch = 0x%02x\r\n", dip_switch);
	serprintf("I2C address = %d\r\n", i2c_address);
	serprintf("sizeof(iocard_data) = %d\r\n", sizeof(iocard_data));

	char *chtxt[] = {
			"SIR1", "FLA1", "SIR2", "FLA2",
			"LCK2", "LCK1", "KPAD", "IR",
			"VOLT", "TAMP", "IR1",  "IR2"
	};

	serprintf("\r\n\r\n");
	for (n = 0; n < 12; n++)
		serprintf(" CH%-2d | ", n);
	serprintf("\r\n");
	for (n = 0; n < 12; n++)
		serprintf(" %4s | ", chtxt[n]);
	serprintf("\r\n");

	timers[DEBUG_PRINT_TIMER] = 1000;
#endif

	uint16_t ad, mA;
	uint8_t ch = 0;

	while(1) {

#ifdef SERIAL_DEBUG_PRINT
		if (timers[DEBUG_PRINT_TIMER] == 0) {
			timers[DEBUG_PRINT_TIMER] = 100;
			debug_print();
		}
#endif
		update_digital_output();

		ad = adc_read(ch);

		add_ad_sample(ch, ad);
		
		if (ch <= 7) {
			mA = ad_to_ma(ad);
			if (!current_control[ch] && mA > 400) {
				over_current |= (1 << ch);
				update_digital_output();
				current_control[ch] = true;
				timers[ch] = 1000;
			}

			if (current_control[ch] && timers[ch] == 0) {
				over_current &= ~(1 << ch);
				current_control[ch] = 0;
				update_digital_output();
			}
		}

		ch++;

		if (ch > 11) {
			ch = 0;
			update_iocard_struct();
			laps++;
		}
	}
}

/*
ISR(USARTC0_RXC_vect) 
{ 
	uint8_t data = USARTC0.DATA;

	if (data >= '0' && data <= '7') {
		data -= '0';
		if (digital_out & (1<<data))
			digital_out &= (~(1<<data));
		else
			digital_out |= (1<<data);
	}
	update_digital_output();
} 
*/
