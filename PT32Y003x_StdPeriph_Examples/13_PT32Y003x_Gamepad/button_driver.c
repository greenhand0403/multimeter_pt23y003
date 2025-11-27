// button_driver.c
#include "button_driver.h"
#include <PT32Y003x_gpio.h>

#define KEY_UP    GPIO_ReadDataBit(GPIOA, GPIO_Pin_1)
#define KEY_DOWN  GPIO_ReadDataBit(GPIOA, GPIO_Pin_2)
#define KEY_LEFT  GPIO_ReadDataBit(GPIOC, GPIO_Pin_3)
#define KEY_RIGHT GPIO_ReadDataBit(GPIOC, GPIO_Pin_4)
#define KEY_A     GPIO_ReadDataBit(GPIOC, GPIO_Pin_5)
#define KEY_B     GPIO_ReadDataBit(GPIOC, GPIO_Pin_6)

void button_init(void)
{
    GPIO_InitTypeDef gpio;
    
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOA, &gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOC, &gpio);
}

uint8_t button_get_state(void)
{
    uint8_t state = 0;
    if (KEY_UP == KEY_PRESSED)     state |= (1 << 0);
    if (KEY_DOWN == KEY_PRESSED)   state |= (1 << 1);
    if (KEY_LEFT == KEY_PRESSED)   state |= (1 << 2);
    if (KEY_RIGHT == KEY_PRESSED)  state |= (1 << 3);
    if (KEY_A == KEY_PRESSED)      state |= (1 << 4);
    if (KEY_B == KEY_PRESSED)      state |= (1 << 5);
    return state;
}