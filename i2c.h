#ifndef I2C_H
#define I2C_H

#include "twi_slave_driver.h"

#define I2C_CMD_REQUEST_ALL   0    /* no payload */
#define I2C_CMD_SET_OUTPUT    1    /* payload: 1 byte mask, 1 byte data */

void i2c_init(uint8_t address);


#endif/*I2C_H*/
