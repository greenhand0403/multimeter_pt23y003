#include "led_driver.h"

// ===== LED初始化 =====
void led_init(void)
{
    GPIO_InitTypeDef gpio;
    
    gpio.GPIO_Pin = GPIO_Pin_4;  // PD4
    gpio.GPIO_Mode = GPIO_Mode_OutPP;
    gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOD, &gpio);
}

// ===== 设置LED状态 =====
void led_set_on(void)
{
    GPIO_SetBits(LED_PIN);
    led_state = 1;
    blink_mode = 0;  // 停止闪烁
}

void led_set_off(void)
{
    GPIO_ResetBits(LED_PIN);
    led_state = 0;
    blink_mode = 0;  // 停止闪烁
}

// ===== 设置LED闪烁模式 =====
void led_set_blink_fast(void)
{
    blink_mode = 1;
    last_blink_time = s_ms_ticks;
}

// ===== LED刷新函数，需要在主循环中调用 =====
void led_update(void)
{
    if (blink_mode == 0) return;
    
    uint32_t current_time = s_ms_ticks;
    if ((current_time - last_blink_time) >= BLINK_FAST_INTERVAL_MS) {
        last_blink_time = current_time;
        led_state = !led_state;
        if (led_state) {
            GPIO_SetBits(LED_PIN);
        } else {
            GPIO_ResetBits(LED_PIN);
        }
    }
}