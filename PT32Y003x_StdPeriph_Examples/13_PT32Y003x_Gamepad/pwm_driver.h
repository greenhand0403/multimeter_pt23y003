// pwm_driver.h
#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "system_config.h"

void pwm_init(void);
void pwm_set_duty(uint16_t duty);

#endif