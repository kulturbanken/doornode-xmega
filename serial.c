#include <avr/io.h>

#include "serial.h"

/* 2MHz 115200 buad 0.08% error */
#define BSEL 11
#define BSCALE -7

void serial_init(int baudrate)
{
	PORTD.DIRSET = PIN3_bm; /* PD3 TXD as output */
	PORTD.DIRCLR = PIN2_bm; /* PD2 RXD as input */

	USARTD0.BAUDCTRLA = BSEL & 0xFF;
	USARTD0.BAUDCTRLB = (BSEL >> 8) | (BSCALE << 4);

	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void serial_send_char(char c)
{
	USARTD0.DATA = c;
	if(!(USARTD0.STATUS & USART_DREIF_bm))
		while(!(USARTD0.STATUS & USART_TXCIF_bm));
	USARTD0.STATUS |= USART_TXCIF_bm;
}

void serial_send_string(char *s)
{
	while(*s) {
		if (*s == '\n')
			serial_send_char('\r');
		serial_send_char(*s);
		s++;
	}
}
