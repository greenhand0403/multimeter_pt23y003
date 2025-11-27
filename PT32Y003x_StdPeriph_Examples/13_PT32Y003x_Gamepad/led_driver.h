// led_driver.h
#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include "system_config.h"

#define BLINK_FAST_INTERVAL_MS 200

void led_init(void);
void led_set_on(void);
void led_set_off(void);
void led_set_blink_fast(void);
void led_update(void);

#endif