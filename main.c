#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stddef.h>

#include "adc.h"
#include "serial.h"
#include "iocard.h"

#define BLINK_DELAY_MS 1000

static void set_32mhz()
{
	CCP = CCP_IOREG_gc;              // disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
	CCP = CCP_IOREG_gc;              // disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void set_output(uint8_t value)
{
	PORTC.OUT = value & 0xFC;
	PORTE.OUT = value & 0x03;
}

int main(void)
{
	//set_32mhz();
	serial_init(0);
        adc_init();

	//PORTA.DIR = (1 << 4);
	PORTC.DIR = 0xFC;
	PORTC.OUT = 0xFC;
	PORTE.DIR = 0x03;
	PORTE.OUT = 0x03;

	uint16_t ad;
	char str[40];

	_delay_ms(100);

	serial_send_string("\r\nStarting up...\r\n");

	int c = 0;

#if 1
	while(1) {
		set_output(1 << c);
		c++;
		if (c > 7)
			c = 0;
		_delay_ms(50);
	}
#else
	while(1) {
		PORTC.OUTTGL = (1 << 5);
		//PORTC.OUTTGL = (1 << 0) | (1 << 1);
		_delay_ms(500);
		ad = adc_read(6);
		//sprintf(str, "degrees_x10: %d\n", degrees_x10);
		sprintf(str, "ADC = %d, mA = %d\n", ad, map(ad, 0, 2048, 0, 3030));
		
		//serial_send_string("Ett till varv...\n");
		serial_send_string(str);
	}
#endif
}
