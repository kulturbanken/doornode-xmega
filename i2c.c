#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>

#include "i2c.h"
#include "twi_slave_driver.h"
#include "iocard.h"

/* TODO: all common handling should be moved to a separate module */
extern iocard_data_t iocard_data;
extern uint8_t digital_out;

TWI_Slave_t twiSlave;      /*!< TWI slave module. */

void TWIC_SlaveProcessData(void)
{
	/* 5 LSB of address byte contains command */
	uint8_t command = (twiSlave.address & 0x1F);

	switch (command) {
	case I2C_CMD_REQUEST_ALL:
		if (twiSlave.bytesReceived == 0) {
			memcpy(&twiSlave.sendData, &iocard_data, sizeof(iocard_data));
			twiSlave.bytesToSend = sizeof(iocard_data);
		}
		break;
	case I2C_CMD_SET_OUTPUT:
		if (twiSlave.bytesReceived == 2) {
			uint8_t mask = twiSlave.receivedData[0];
			uint8_t data = twiSlave.receivedData[1];
			uint8_t setbits = data & mask;
			uint8_t clrbits = (~data) & mask;

			digital_out |= setbits;
			digital_out &= ~clrbits;

			/* We've got all we need, return a NACK */
			twiSlave.abort = true;
		}
		
		break;
	default:
		/* Unhandled command */
		twiSlave.abort = true;
	}
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

void i2c_init(uint8_t address)
{
	/* Only use first 2 MSB for address matching */
	/* NOTE: I2C/TWI uses 7 bit addresses! */

	uint8_t addrmask = ~(0x03 << 5); /* 1's in the mask is ignored during match */

	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, address, addrmask, TWI_SLAVE_INTLVL_LO_gc);
}
