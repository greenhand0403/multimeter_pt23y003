// button_driver.h
#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include "system_config.h"

// ===== 按键定义 =====
#define KEY_UP      GPIO_ReadDataBit(GPIOA, GPIO_Pin_1)   // PA1
#define KEY_LEFT    GPIO_ReadDataBit(GPIOA, GPIO_Pin_2)   // PA2
#define KEY_DOWN    GPIO_ReadDataBit(GPIOC, GPIO_Pin_3)   // PC3
#define KEY_RIGHT   GPIO_ReadDataBit(GPIOC, GPIO_Pin_4)   // PC4
#define KEY_A       GPIO_ReadDataBit(GPIOC, GPIO_Pin_5)   // PC5
#define KEY_B       GPIO_ReadDataBit(GPIOC, GPIO_Pin_6)   // PC6

// 按键有效电平：低电平有效
#define KEY_PRESSED 0

void button_init(void);
void button_deinit(void);
uint8_t button_get_state(void);

#endif