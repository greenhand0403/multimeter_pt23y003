// servo_driver.h
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "system_config.h"

void servo_init(void);
void servo_set_angle(uint8_t angle);

#endif