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

static uint8_t dip_switch;
static uint8_t i2c_address;
static uint8_t over_current = 0;
uint8_t digital_out = 0;

static char str[128]; /* printf temporary buffer */
#define serprintf(fmt, ...) sprintf(str, fmt, ## __VA_ARGS__); serial_send_string(str)

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

	/* 2 LSB is on PORTE, rest on PORTC */
	PORTE.OUT = (PORTE.OUT & (~0x03)) | (out_byte & (0x03));
	PORTC.OUT = (out_byte & (~0x03)) | (PORTC.OUT & 0x03);
}

#if 0
static void set_digital_output_pin(uint8_t pin)
{
	if (pin < 2) {
		PORTE.OUTSET = (1 << pin);
	} else {
		PORTC.OUTSET = (1 << pin);
	}
}

static void clr_digital_output_pin(uint8_t pin)
{
	if (pin < 2) {
		PORTE.OUTCLR = (1 << pin);
	} else {
		PORTC.OUTCLR = (1 << pin);
	}
}
#endif

#define AD_BUF 4
static struct cctrl_struct {
	uint8_t  triggered;
	int16_t adval[AD_BUF];
} cctrl[12];

iocard_data_t iocard_data;

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
	uint16_t ret = 0;

	for (n = 0; n < AD_BUF; n++)
		ret += ch->adval[n];
	ret /= AD_BUF;

	return ret;
}

static void update_iocard_struct()
{
	uint8_t n;
	
	cli();
	iocard_data.version = 1;
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

int main(void)
{
	//set_32mhz();

	PORTA.DIR = 0x00;
	PORTC.DIR = 0xFC;
	PORTC.OUT = 0xFC;
	PORTD.DIR = 0x00;
	PORTCFG.MPCMASK = 0xFF; /* "select" all pins */
	PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm; /* enable pull-up and inverted input on selected pins */
	PORTE.DIR = 0x03;
	PORTE.OUT = 0x03;
	PORTR.DIR = 0x00;

	_delay_ms(1);

	dip_switch = PORTD.IN;
	i2c_address = dip_switch >> 6;

	/* Reset DIP pins for serial use etc */
	PORTCFG.MPCMASK = 0xFF;
	PORTD.PIN0CTRL = 0;

	serial_init(0);
        adc_init();
	timer_init();
	i2c_init(i2c_address);

	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; 
	sei();

	_delay_ms(100);

	serprintf("\r\nStarting up...\r\n");
	serprintf("DIP switch = 0x%02x\r\n", dip_switch);
	serprintf("I2C address = %d\r\n", i2c_address);
	serprintf("sizeof(iocard_data) = %d\r\n", sizeof(iocard_data));

	uint16_t ad, mA, low = 9999, high = 0, laps = 0, avg, flipflop = 0;
	uint8_t ch = 0, skip_next, n;

	timers[8] = 1000;

	update_digital_output();
	
	while(1) {
		if (timers[8] == 0) {
			timers[8] = 100;
			if (flipflop == 1) {
				flipflop = 0;
			} else {
				avg = 0;
				for (n = 0; n < AD_BUF; n++)
					avg += cctrl[2].adval[n];
				avg /= AD_BUF;

				if (avg < low)
					low = avg;
				if (avg > high)
					high = avg;
				serprintf("CH2 = %-5d low = %-5d high = %-5d diff = %-3d | laps = %d Din = 0x%02x mA = %d digital_out = 0x%02x\r\n",
					  avg, low, high, high - low, laps, get_digital_in(), ad_to_ma(avg), digital_out);
				flipflop = 1;
			}
			laps = 0;
		}

		ad = adc_read(ch);

		for (n = 0; n < AD_BUF-1; n++)
			cctrl[ch].adval[n] = cctrl[ch].adval[n+1];
		cctrl[ch].adval[AD_BUF-1] = ad;
		
		skip_next = 0;
		if (ch <= 7) {
			mA = ad_to_ma(ad);
			if (!cctrl[ch].triggered && mA > 400) {
				over_current |= (1 << ch);
				update_digital_output();
				//clr_digital_output_pin(ch);

				cctrl[ch].triggered = 1;
				timers[ch] = 1000;

				sprintf(str, "Over current (%d mA) detected on ch %d!\r\n", mA, ch);
				serial_send_string(str);
			}

			if (cctrl[ch].triggered && timers[ch] == 0) {
				over_current &= ~(1 << ch);

				cctrl[ch].triggered = 0;
				skip_next = 1;

				sprintf(str, "Re-activating output on ch %d.\r\n", ch);
				serial_send_string(str);

				update_digital_output();
				//set_digital_output_pin(ch);
			}
		}

		if (!skip_next)
			ch++;

		if (ch > 11) {
			ch = 0;
			update_iocard_struct();
			laps++;
		}
	}
}

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
