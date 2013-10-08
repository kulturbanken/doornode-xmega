#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdio.h>

#define serprintf(fmt, ...) { char str[128]; sprintf(str, fmt, ## __VA_ARGS__); serial_send_string(str); }

void serial_init(int baudrate);
void serial_send_char(char c);
void serial_send_string(char *s);

#endif
