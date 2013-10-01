#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stddef.h>

#include "adc.h"
#include "serial.h"
#include "iocard.h"
#include "timer.h"

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

static void set_pin(uint8_t pin)
{
	if (pin < 2) {
		PORTE.OUTSET = (1 << pin);
	} else {
		PORTC.OUTSET = (1 << pin);
	}
}

static void clr_pin(uint8_t pin)
{
	if (pin < 2) {
		PORTE.OUTCLR = (1 << pin);
	} else {
		PORTC.OUTCLR = (1 << pin);
	}
}

static struct {
	uint8_t  triggered;
//	uint16_t adval;
} cctrl[12];

void timer_callback()
{
}

int main(void)
{
	//set_32mhz();
	serial_init(0);
        adc_init();
	timer_init(*timer_callback);

	//PORTA.DIR = (1 << 4);
	PORTC.DIR = 0xFC;
	PORTC.OUT = 0xFC;
	PORTE.DIR = 0x03;
	PORTE.OUT = 0x03;
	PORTC.OUTCLR = (1<<5);

	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; 
	sei(); // Interrupts aktivieren
	
	int16_t ad;
	char str[40];

	_delay_ms(100);

	serial_send_string("\r\nStarting up...\r\n");

	int16_t mA;
	uint8_t ch = 0, skip_next;

	timers[8] = 1000;

	while(1) {
		if (timers[8] == 0) {
			timers[8] = 1000;
			serial_send_string("1 sec...\r\n");
		}

		ad = adc_read(ch);

		//cctrl[ch].adval = ad;
		skip_next = 0;
		if (ch <= 7) {
			mA = map(ad, 140, 662, 200, 947);
			if (!cctrl[ch].triggered && mA > 400) {
				clr_pin(ch);

				cctrl[ch].triggered = 1;
				timers[ch] = 1000;

				sprintf(str, "Over current (%d mA) detected on ch %d!\r\n", mA, ch);
				serial_send_string(str);
			}

			if (cctrl[ch].triggered && timers[ch] == 0) {
				cctrl[ch].triggered = 0;
				skip_next = 1;

				sprintf(str, "Re-activating output on ch %d.\r\n", ch);
				serial_send_string(str);

				set_pin(ch);
			}
		}

		if (!skip_next)
			ch++;
	}
}
