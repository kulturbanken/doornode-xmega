#include <avr/io.h>

#include "serial.h"

/* CPU@2MHz 115200 buad 0.08% error */
#define BSEL 11
#define BSCALE -7

#define USART USARTC0
#define PORT  PORTC

void serial_init(int baudrate)
{
	PORT.DIRSET = PIN3_bm; /* Px3 TXD as output */
	PORT.DIRCLR = PIN2_bm; /* Px2 RXD as input */

	USART.BAUDCTRLA = BSEL & 0xFF;
	USART.BAUDCTRLB = (BSEL >> 8) | (BSCALE << 4);

	USART.CTRLA = USART_RXCINTLVL_LO_gc; 
	USART.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void serial_send_char(char c)
{
	USART.DATA = c;
	if(!(USART.STATUS & USART_DREIF_bm))
		while(!(USART.STATUS & USART_TXCIF_bm));
	USART.STATUS |= USART_TXCIF_bm;
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
