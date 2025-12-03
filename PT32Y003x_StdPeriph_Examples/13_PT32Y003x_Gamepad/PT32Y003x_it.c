#include "PT32Y003x.h"
#include "PT32Y003x_uart.h"
#include <PT32Y003x_gpio.h>

#define RX_RING_SIZE 128

volatile uint8_t rx_ring[RX_RING_SIZE] = {0};
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

// ===== UART0中断服务函数 =====
void UART0_Handler(void)
{
    if(UART_GetFlagStatus(UART0,UART_FLAG_RXNE))
	{
		uint8_t b = (uint8_t)UART_ReceiveData(UART0);
        uint16_t next = (rx_head + 1) % RX_RING_SIZE;

        if (next == rx_tail) {
            // buffer full: 丢弃这个字节（或选择丢 oldest）
            // 可选统计溢出次数以便调试
            // rx_overflow_count++;
        } else {
            rx_ring[rx_head] = b;
            rx_head = next;
            // 可选：如果想在ISR里快速检测到 \n，可检查 b == '\n' 并设置标志，
            // 但仍建议让主循环来做行拼接与处理，避免ISR里复杂处理
        }
	}
}

void I2C0_Handler(void)
{
    // 预留：MPU6050中断处理
}

extern volatile uint32_t s_ms_ticks;   // 1ms 计数（全局）
extern volatile uint32_t s_ms_delay;   // 阻塞式 ms 延时用
// SysTick 中断：1ms 心跳 + 阻塞延时递减
/**
* @brief SysTick中断服务函数
* @param None
* @retval None
*/
void SysTick_Handler(void)
{
  s_ms_ticks++;
  if (s_ms_delay!= 0x00) 
    s_ms_delay--;
}

// 当任意按键按下时唤醒
void EXTIA_Handler(void)
{
    // 清除中断标志
    EXTI_ClearFlag(EXTIA, GPIO_Pin_1);
    EXTI_ClearFlag(EXTIA, GPIO_Pin_2);
}

void EXTIC_Handler(void)
{
    // 清除中断标志
    EXTI_ClearFlag(EXTIC, GPIO_Pin_3);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_4);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_5);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_6);
}