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

volatile uint8_t digital_out = 0;
volatile iocard_data_t iocard_data;

#if 0
static void set_32mhz()
{
	CCP = CCP_IOREG_gc;              // disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
	CCP = CCP_IOREG_gc;              // disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

#define AD_BUF 16
static struct cctrl_struct {
	uint8_t  triggered;
	int16_t adval[AD_BUF];
} cctrl[12];

static inline uint8_t get_digital_in()
{
	uint8_t ret = 0;

	/* PR0 + PR1 on bit 0 and 1 respectively */
	ret |= PORTR.IN & 0x03;

	/* PE3 on bit 3 */
	ret |= PORTE.IN & (1<<3);

	return ret;
}

static inline uint16_t ad_to_ma(uint16_t ad)
{
	return map(ad, 140, 662, 200, 947);
}

static inline uint16_t get_analog_avg(struct cctrl_struct *ch)
{
	uint8_t n;
	uint32_t ret = 0;

	for (n = 0; n < AD_BUF; n++)
		ret += ch->adval[n];
	ret /= AD_BUF;

	return (uint16_t)ret;
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
		iocard_data.analog_in[n] = get_analog_avg(&cctrl[n]);
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
			serprintf("%5d | ", get_analog_avg(&cctrl[n]));
		}
		serprintf("%5d", laps);
#if 0
		avg = get_analog_avg(&cctrl[2]);
		if (avg < low)
			low = avg;
		if (avg > high)
			high = avg;
		serprintf("CH2 = %-5d low = %-5d high = %-5d diff = %-3d | PORTD = 0x%02x laps = %d Din = 0x%02x mA = %d digital_out = 0x%02x\r\n",
			  avg, low, high, high - low, PORTD.IN, laps, get_digital_in(), ad_to_ma(avg), digital_out);
#endif
		flipflop = 1;
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

	/* Enable 8 second watchdog, will only reset on I2C requests */
	WDT_EnableAndSetTimeout( WDT_PER_8KCLK_gc );

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

		for (n = 0; n < AD_BUF-1; n++)
			cctrl[ch].adval[n] = cctrl[ch].adval[n+1];
		cctrl[ch].adval[AD_BUF-1] = ad;
		
		if (ch <= 7) {
			mA = ad_to_ma(ad);
			if (!cctrl[ch].triggered && mA > 400) {
				over_current |= (1 << ch);
				update_digital_output();
				cctrl[ch].triggered = 1;
				timers[ch] = 1000;
			}

			if (cctrl[ch].triggered && timers[ch] == 0) {
				over_current &= ~(1 << ch);
				cctrl[ch].triggered = 0;
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
