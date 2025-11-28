#include "system_config.h"
#include <PT32Y003x_pwr.h>
#include <PT32Y003x_exti.h>

// 蓝牙名称是否合法 1 则合法
uint8_t BLE_NAME_LEGAL = 0;
extern uint8_t Legal_MAC[2];

// 手柄当前的工作模式，影响手柄数据包代表的含义
// 模式一代表按键舵机模式，当触发按键中断时记录发送标志位，发送完按键数据后清除标志位
// 模式二代表陀螺仪模式，当触发定时器中断 (每20ms) 时记录发送标志位，发送完陀螺仪数据后清除标志位
volatile work_mode_t g_work_mode = WORK_MODE_IDLE;
volatile work_mode_t g_work_mode_prev = WORK_MODE_IDLE;

extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;
extern volatile uint8_t rx_ring[128];
// 复制串口接收区用的临时行缓冲
#define LINE_BUF_SIZE 128
char line_buf[LINE_BUF_SIZE];
uint16_t line_len = 0;

// ===== 外部函数声明 =====
extern void led_init(void);
extern void led_set_blink_fast(void);  // 只保留快闪
extern void led_set_on(void);
extern void led_set_off(void);
extern void led_update(void);

extern void button_init(void);

extern void bluetooth_init(void);

extern void bluetooth_configure_name_start(void);

// ===== 处理蓝牙响应 =====
extern void ProcessBluetoothResponse(const char *line);

#pragma region 串口格式化输出辅助函数
// 向指定UART发送字符串
void UART_SendString(UART_TypeDef* UARTx, const char *str)
{
    while (*str) {
        UART_SendData(UARTx, *str);
        while (UART_GetFlagStatus(UARTx, UART_FLAG_TXE) == RESET);
        str++;
    }
}
// 最简单的数字转字符串（仅正整数）
// static void utoa_simple(unsigned int val, char *buf) {
//     char tmp[12];
//     int i = 0;
//     if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return; }
//     while (val) {
//         tmp[i++] = '0' + (val % 10);
//         val /= 10;
//     }
//     int j;
//     for (j = 0; j < i; ++j) buf[j] = tmp[i - 1 - j];
//     buf[i] = '\0';
// }

// void UART_Printf_Simple(UART_TypeDef* UARTx, const char *str, unsigned int num) {
//     // 仅示例：把 str 发出，再发 num 的字符串（用于替代格式化）
//     UART_SendString(UARTx, str);
//     char tmp[32];
//     utoa_simple(num, tmp);
//     UART_SendString(UARTx, tmp);
//     UART_SendString(UARTx, "\r\n");
// }

#pragma endregion

// ===== 模式检测函数 =====
work_mode_t detect_work_mode(void)
{
    GPIO_InitTypeDef gpio;
    
    // 配置PA3为输入上拉
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOA, &gpio);
    
    delay_ms(20);
    
    if (GPIO_ReadDataBit(GPIOA, GPIO_Pin_3) == 1) {
        return WORK_MODE_1_SERVO;  // 悬空 - 模式1
    } else {
        return WORK_MODE_2_GYRO;   // 接地 - 模式2
    }
}
void uart1_init(void)
{
    // UART1配置，用于调试
    UART_InitTypeDef uart;

    GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_1, AFIO_AF_1, ENABLE);

    uart.UART_BaudRate = DEBUG_BAUD;
    uart.UART_WordLengthAndParity = UART_WordLengthAndParity_8D;
    uart.UART_StopBitLength = UART_StopBitLength_1;
    uart.UART_ParityMode = UART_ParityMode_Even;
    uart.UART_Receiver = UART_Receiver_Disable;
    uart.UART_LoopbackMode = UART_LoopbackMode_Disable;

    UART_Init(UART1, &uart);
    UART_Cmd(UART1, ENABLE);

}
// ===== 初始化所有模块 =====
void system_init(void)
{
    // 系统时钟定时器初始化，用于延时和定时任务
    SysTick_Init();

    // UART1初始化，用于调试
    uart1_init();

    // 初始化外部 LED 灯，PD4
    led_init();
    // 快闪模式
    led_set_blink_fast();

    // 手柄六个按键初始化
    button_init();
    // 蓝牙模块初始化
    bluetooth_init();
    
    // 需要读取PD3判断蓝牙是否连接，所以配置一下管脚
    GPIO_InitTypeDef gpio;
    // PD3
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOD, &gpio);
}

// ===== 休眠功能 =====
void enter_sleep_mode(void)
{
    led_set_off();
    
    // 配置任意按键唤醒
    EXTI_TriggerTypeConfig(EXTIA, GPIO_Pin_1, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIA, GPIO_Pin_2, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_3, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_4, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_5, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_6, EXTI_Trigger_RisingFalling);
    
    EXTI_ITConfig(EXTIA, GPIO_Pin_1, ENABLE);
    EXTI_ITConfig(EXTIA, GPIO_Pin_2, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_3, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_4, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_5, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_6, ENABLE);
    
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = EXTIA_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x00;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = EXTIC_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x00;
    NVIC_Init(&nvic);
    
    PWR_EnterDeepSleepMode(PWR_DeepSleepEntry_WFI);
}
void PollAndProcessUARTLines(void)
{
    // 从环形缓冲读取字节，拼成行
    while (rx_tail != rx_head) {
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % 128;

        // 限制行长度，防止越界
        if (line_len < LINE_BUF_SIZE - 1) {
            line_buf[line_len++] = (char)b;
        } else {
            // 行太长，丢弃并重置
            line_len = 0;
        }

        // 检测到 \r\n 结尾（常见的是 \r\n 两字节）
        if (b == '\n') {
            // 去掉末尾可能的 \r
            if (line_len >= 2 && line_buf[line_len - 2] == '\r') {
                line_buf[line_len - 2] = '\0';
            } else {
                line_buf[line_len - 1] = '\0';
            }

            // 设置标志或直接调用处理函数
            ProcessBluetoothResponse(line_buf); // 建议把行内容传给处理函数
            line_len = 0;
        }
    }
}
// ===== 主函数 =====
int main(void)
{
    system_init();
    
    // 状态机一，等待查询到蓝牙名称合法
    while (!BLE_NAME_LEGAL)
    {
        // 未连接LED快闪
        led_update();
        // 串口接收数据按行读取
        PollAndProcessUARTLines();
        // 蓝牙名称检查循环
        bluetooth_configure_name_start();
    }
    UART_SendString(UART1, "BLE NAME OK\r\n");
    led_set_on();
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(u8* file, u32 line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
	/* Infinite loop */
	while (1)
	{
	}
}
#endif