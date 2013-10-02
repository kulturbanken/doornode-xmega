#ifndef TIMER_H
#define TIMER_H

#include <avr/io.h>
#include <inttypes.h>

extern uint16_t timers[];

void timer_init();

#endif
