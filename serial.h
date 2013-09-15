#ifndef _SERIAL_H
#define _SERIAL_H

void serial_init(int baudrate);
void serial_send_char(char c);
void serial_send_string(char *s);

#endif
