#include "PT32Y003x.h"
#include "PT32Y003x_uart.h"

// ===== 外部变量声明 =====
extern u8 rx_buffer[64];
extern u16 rx_index;
extern u8 bluetooth_ready;

// ===== 处理蓝牙响应 =====
void ProcessBluetoothResponse(void)
{
    // 检查是否是OK响应
    if (rx_index >= 4 && 
        rx_buffer[0] == 'O' && 
        rx_buffer[1] == 'K' && 
        rx_buffer[2] == 0x0D && 
        rx_buffer[3] == 0x0A)
    {
        bluetooth_ready = 1;
        // 可以在这里添加蓝牙连接成功的处理
    }
    else
    {
        // 处理其他蓝牙响应
        // 例如：连接状态、错误信息等
    }
}

// ===== UART0中断服务函数 =====
void UART0_IRQHandler(void)
{
    if (UART_GetITStatus(UART0, UART_IT_RXNEI) != RESET)
    {
        // 读取接收到的数据
        u8 received_byte = UART_ReceiveData(UART0);
        
        // 将数据存入缓冲区
        if (rx_index < 63) // 防止缓冲区溢出
        {
            rx_buffer[rx_index++] = received_byte;
            
            // 检查是否接收到回车换行（0x0D 0x0A）
            if (received_byte == 0x0A && rx_index >= 2 && rx_buffer[rx_index-2] == 0x0D)
            {
                // 处理接收到的完整行数据
                ProcessBluetoothResponse();
                rx_index = 0; // 清空缓冲区
            }
        }
        else
        {
            // 缓冲区满，清空
            rx_index = 0;
        }
        
        // 清除中断标志
        UART_ClearFlag(UART0, UART_IT_RXNEI);
    }
}

// ===== 其他中断服务函数预留 =====
void UART1_IRQHandler(void)
{
    // UART1当前只用于调试输出，无需接收中断
    if (UART_GetITStatus(UART1, UART_IT_RXNEI) != RESET)
    {
        // 读取并丢弃数据
        UART_ReceiveData(UART1);
        UART_ClearFlag(UART1, UART_IT_RXNEI);
    }
}

void I2C0_IRQHandler(void)
{
    // 预留：MPU6050中断处理
}

// 其他外设中断函数可根据需要添加

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
  if (s_ms_delay) s_ms_delay--;
}