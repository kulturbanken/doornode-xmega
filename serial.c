#include <avr/io.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "usart_driver.h"

/* CPU@2MHz 115200 buad 0.08% error */
#define BSEL 11
#define BSCALE -7

#define USART USARTC0
#define PORT  PORTC

USART_data_t USART_data;

static bool serial_is_initiated = 0;

void serial_init(int baudrate)
{
	PORT.DIRSET = PIN3_bm; /* Px3 TXD as output */
	PORT.DIRCLR = PIN2_bm; /* Px2 RXD as input */

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 115200 */
	USART_Baudrate_Set(&USART, BSEL, -7);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/*
	USART.BAUDCTRLA = BSEL & 0xFF;
	USART.BAUDCTRLB = (BSEL >> 8) | (BSCALE << 4);

	USART.CTRLA = USART_RXCINTLVL_LO_gc; 
	USART.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	*/

	serial_is_initiated = 1;
}

void serial_send_char(char c)
{
	if (!serial_is_initiated)
		return;
	/*
	USART.DATA = c;
	if(!(USART.STATUS & USART_DREIF_bm))
		while(!(USART.STATUS & USART_TXCIF_bm));
	USART.STATUS |= USART_TXCIF_bm;
	*/
	USART_TXBuffer_PutByte(&USART_data, c);
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


/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&USART_data);
}


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}
