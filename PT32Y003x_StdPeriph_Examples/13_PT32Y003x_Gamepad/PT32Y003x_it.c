#include "PT32Y003x.h"
#include "PT32Y003x_uart.h"
#include <PT32Y003x_gpio.h>

// ===== 外部变量声明 =====
extern u16 rx_buffer[64];
extern u8 rx_index;

// ===== UART0中断服务函数 =====
void UART0_Handler(void)
{
    if(UART_GetFlagStatus(UART0,UART_FLAG_RXNE))
	{
		rx_buffer[rx_index++]=UART_ReceiveData(UART0);
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