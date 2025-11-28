#ifndef __DELAY_H__
#define __DELAY_H__

// #include <stdint.h>
#include "PT32Y003x.h"
#include "core_cm0.h"
#include <PT32Y003x_rcc.h>

// ===== SysTick 延时 =====
extern volatile uint32_t s_ms_ticks;   // 1ms 计数（全局）
extern volatile uint32_t s_ms_delay;   // 阻塞式 ms 延时用
// 基于 SysTick 定时器
void SysTick_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif
