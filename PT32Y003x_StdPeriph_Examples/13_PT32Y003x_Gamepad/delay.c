#include "delay.h"
volatile uint32_t s_ms_ticks = 0;   // 定义一次
volatile uint32_t s_ms_delay = 0;   // 定义一次

// 配置为 1kHz（1ms）中断
void SysTick_Init_1kHz(void)
{
    s_ms_ticks = 0;
    // 前面已经设置了系统时钟48M分频8所以HCLK变成6M
    uint32_t hclk = RCC_GetClockFreq(RCC_HCLK);        // 参考 Systick_LED 的写法
    // 1kHz: 每 1ms 进一次 SysTick_Handler
    if (SysTick_Config(hclk / 1000U)) { while (1) {} } // 出错直接卡死，便于发现问题
}

// 阻塞式毫秒延时（不影响 us 定时）
void delay_ms(uint32_t ms)
{
    s_ms_delay = ms;
    // 确保计数器已运行（避免低功耗退出后状态异常）
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    while (s_ms_delay) { /* 等待 */ }
}

// 微秒级延时：利用当前 1kHz SysTick 的 VAL 差分计数
// 原理：SysTick 以 hclk 频率递减；我们读取起点 VAL，轮询已消耗的时钟周期数，达到 us 对应的周期数退出
void delay_us(uint32_t us)
{
    uint32_t hclk = RCC_GetClockFreq(RCC_HCLK);
    uint32_t cycles = (hclk / 1000000U) * us;   // 需要的 HCLK 周期数
    uint32_t start = SysTick->VAL;              // 起点
    uint32_t load  = SysTick->LOAD + 1U;        // 计数器模值（+1 因为从 LOAD 到 0 共 load+1 个周期）
    uint32_t elapsed = 0;

    // 注意：SysTick 递减并在 0 时重装载，因此用取模差分计算已过周期
    while (elapsed < cycles)
    {
        uint32_t now = SysTick->VAL;
        uint32_t diff = (start >= now) ? (start - now) : (start + (load - now));
        elapsed += diff;
        start = now;
    }
}