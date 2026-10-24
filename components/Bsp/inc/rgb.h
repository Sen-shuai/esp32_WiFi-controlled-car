#ifndef _RGB_H_
#define _RGB_H_


#include "driver/rmt_tx.h"
#include "led_strip.h"


void rgb_init(void);
void rgb_set_color(uint8_t red, uint8_t green, uint8_t blue);


#endif

