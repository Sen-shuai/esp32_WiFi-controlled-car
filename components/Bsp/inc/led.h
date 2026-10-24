#ifndef _LED_H
#define _LED_H


#include "driver/gpio.h"



#define LED_PIN 2

#define LED(x) do{x ? gpio_set_level(LED_PIN, 1) : gpio_set_level(LED_PIN, 0);}while(0)
#define LED_toggle() do{gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));}while(0)


void led_init(void);

#endif
