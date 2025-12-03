// pwm_driver.h
#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "system_config.h"

void pwm_init(void);
void pwm_set_duty(uint16_t duty); // duty: 0 - 1000, ¶ÔÓ¦ 0% - 100%

#endif