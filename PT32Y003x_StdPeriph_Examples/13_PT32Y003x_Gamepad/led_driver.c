// led_driver.c
#include "led_driver.h"
#include "system_config.h"
#include <PT32Y003x_gpio.h>
#include "delay.h"

static uint32_t last_blink_time = 0;
static uint8_t led_state = 0;
static uint8_t blink_mode = 0;

void led_init(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OutPP;
    gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOD, &gpio);
    led_set_off();
}

void led_set_on(void)
{
    GPIO_SetBits(LED_PIN);
    led_state = 1;
    blink_mode = 0;
}

void led_set_off(void)
{
    GPIO_ResetBits(LED_PIN);
    led_state = 0;
    blink_mode = 0;
}

void led_set_blink_fast(void)
{
    blink_mode = 1;
    last_blink_time = s_ms_ticks;
}

void led_update(void)
{
    if (!blink_mode) return;
    if ((s_ms_ticks - last_blink_time) >= BLINK_FAST_INTERVAL_MS) {
        last_blink_time = s_ms_ticks;
        led_state = !led_state;
        if (led_state) GPIO_SetBits(LED_PIN);
        else           GPIO_ResetBits(LED_PIN);
    }
}