#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>

volatile uint16_t timers[16];

ISR(TCC0_OVF_vect)
{
	PORTC.OUTTGL = (1<<5);

	uint8_t n;

	for (n = 0; n < 16; n++)
		if (timers[n] > 0)
			timers[n] -= 10;

	//callback();
}

void timer_init()
{
	uint8_t n;

	TCC0.CTRLA = TC_CLKSEL_DIV8_gc; // Prescale by 2 = 1MHz @ 2MHz sysclk
	TCC0.CTRLB = 0x00;              // select mode: Normal
	TCC0.PER = 2500;                // 1000 Hz
	TCC0.CNT = 0x00;                // Reset counter
	TCC0.INTCTRLA = 0b00000011;     // Interrupt Highlevel

	for (n = 0; n < 16; n++)
		timers[n] = 0;
}
