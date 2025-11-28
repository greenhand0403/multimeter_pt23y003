// button_driver.c
#include "button_driver.h"

void button_init(void)
{
    // 配置所有按键引脚为输入上拉（低电平有效）
    GPIO_InitTypeDef gpio;
    
    // PA1, PA2
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOA, &gpio);
    
    // PC3~PC6
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOC, &gpio);
}
void button_deinit(void)
{
    // 配置所有按键引脚为浮空输入（释放资源）
    GPIO_InitTypeDef gpio;
    
    // PA1, PA2
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOA, &gpio);
    
    // PC3~PC6
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOC, &gpio);
}

// 返回一个字节，每位代表一个按键状态（1=按下）
uint8_t button_get_state(void)
{
    uint8_t state = 0;
    if (KEY_UP == KEY_PRESSED)     state |= (1 << 0);  // UP
    if (KEY_DOWN == KEY_PRESSED)   state |= (1 << 1);  // DOWN
    if (KEY_LEFT == KEY_PRESSED)   state |= (1 << 2);  // LEFT
    if (KEY_RIGHT == KEY_PRESSED)  state |= (1 << 3);  // RIGHT
    if (KEY_A == KEY_PRESSED)      state |= (1 << 4);  // A
    if (KEY_B == KEY_PRESSED)      state |= (1 << 5);  // B
    return state;
}