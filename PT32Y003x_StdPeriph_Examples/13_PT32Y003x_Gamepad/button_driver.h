// button_driver.h
#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include "system_config.h"

void button_init(void);
uint8_t button_get_state(void); // 返回当前按键状态（位掩码）

#endif