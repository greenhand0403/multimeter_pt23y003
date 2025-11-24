#include "system_config.h"
#include <PT32Y003x_pwr.h>
#include <PT32Y003x_exti.h>
// 头文件的添加在keil里面设置了，自动搜索头文件

// ===== 全局变量 =====
// 指令类别 01:手柄建立连接 02:手柄状态数据
volatile uint8_t g_current_key_state = 0;
// 手柄当前的工作模式，影响手柄数据包代表的含义
// 模式一代表按键舵机模式，当触发按键中断时记录发送标志位，发送完按键数据后清除标志位
// 模式二代表陀螺仪模式，当触发定时器中断 (每20ms) 时记录发送标志位，发送完陀螺仪数据后清除标志位
volatile work_mode_t g_work_mode = WORK_MODE_IDLE;
volatile uint8_t g_seq_num = 0;  // 指令流水号
volatile uint32_t g_last_packet_time = 0;  // 上次发送包的时间戳，用于每20ms发送陀螺仪数据的 逻辑

extern uint16_t rx_buffer[64];
extern uint8_t rx_index;

// ===== 外部函数声明 =====
extern void led_init(void);
extern void led_set_blink_fast(void);  // 只保留快闪
extern void led_set_on(void);
extern void led_set_off(void);
extern void led_update(void);

extern void button_init(void);
extern uint8_t button_get_state(void);
extern void button_check_wakeup(void);

extern void servo_init(void);
extern void servo_set_angle(uint8_t angle);

extern void bluetooth_init(void);
extern void bluetooth_send_packet(protocol_packet_t* packet);
extern void bluetooth_check_connection(void);
extern uint8_t bluetooth_check_sleep_timeout(void);
extern void bluetooth_configure_name(void);
// ===== 处理蓝牙响应 =====
extern void ProcessBluetoothResponse(void);

extern void gyro_init(void);

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

// 专门输出到UART0（蓝牙）
void Bluetooth_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    UART_SendString(UART0, buffer);
}

// 专门输出到UART1（调试）
void Debug_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    UART_SendString(UART1, buffer);
}
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
    
    // UART1初始化
    uart1_init();

    // 正常发送工作状态信息
    // Debug_Printf("mode:%d\r\n", g_work_mode);
    
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

// ===== 发送连接状态包 =====
void send_connect_packet(void)
{
    protocol_packet_t packet;
    
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_CONNECT;
    
    // MAC地址后2字节（实际应从蓝牙模块读取）
    packet.data[0] = 0xAA;
    packet.data[1] = 0xBB;
    packet.data[2] = 0x00;
    packet.data[3] = 0x00;
    packet.data[4] = 0x00;
    packet.data[5] = 0x00;
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + (packet.data[0] + packet.data[1] + 
                packet.data[2] + packet.data[3] + packet.data[4] + 
                packet.data[5]) + packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
}

// ===== 发送按键状态包 =====
void send_key_status_packet(void)
{
    protocol_packet_t packet;
    
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_STATUS;
    
    packet.data[0] = g_current_key_state;
    packet.data[1] = 0x00;
    packet.data[2] = 0x00;
    // 陀螺仪数据（模式2时使用）
    packet.data[3] = 0x5A;
    packet.data[4] = 0x5A;
    packet.data[5] = 0x5A;
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + (packet.data[0] + packet.data[1] + 
                packet.data[2] + packet.data[3] + packet.data[4] + 
                packet.data[5]) + packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
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

// ===== 主函数 =====
int main(void)
{
    uint8_t i = 0;

    system_init();
    
    // TODO: 检查并配置蓝牙名称
    bluetooth_configure_name();
    // 等待直到蓝牙指令查询返回正确格式的蓝牙名称
    while (rx_buffer[4]!='N')
    {
        // 可选，可以试一下默认的蓝牙名称，恢复出厂设置试试
        if (rx_index>=14)
        {
            ProcessBluetoothResponse();
        }
        
        // 接收蓝牙模块发送到串口0的数据，转发到串口1调试
        while(rx_index)
        {
            UART_SendData(UART1,rx_buffer[i++]);
            if(i==rx_index)
            {
                i=0;
                rx_index=0;
            }
        }

        // LED闪烁表示蓝牙名称不正确
        led_update();
        delay_ms(20);
    }
    // TODO: 测试，走到这里说明前面的蓝牙名称判断逻辑已经走通
    led_set_on();

    while (1)
    {
        // LED闪烁表示未连接蓝牙
        // led_update();

        // 接收蓝牙模块发送到串口0的数据，转发到串口1调试
        while(rx_index)
		{
			UART_SendData(UART1,rx_buffer[i++]);
			if(i==rx_index)
			{
				i=0;
				rx_index=0;
			}
		}

        delay_ms(20);
    }
    
    // LED状态灯处理
    // 先发 AT+CF00 设置蓝牙模块的LED灯状态指示为默认显示模式，未连接LED低电平熄灭，连接高
    // 再由 MCU读 PD3 持续判断蓝牙连接状态，若无连接，120秒自动低功耗休眠
    // 进入休眠时开启外部按键中断，用于检测按键唤醒
    // MCU 持续持续判断蓝牙连接状态，直到主机连接蓝牙模块，然后MCU取消自动休眠，并发送首包确认连接
    // send_connect_packet();
    
    while (1)
    {
        // TODO: 工作主循环，先检查有没有初始化工作状态，需要额外一个的变量来记录一个工作模式是否初始化完成，当用户在使用过程中切换到新的工作模式时，就重新初始化工作状态并再次记录
        
        // 检测工作模式
        g_work_mode = detect_work_mode();

        // 根据模式初始化相应模块
        if (g_work_mode == WORK_MODE_1_SERVO) {
            servo_init();
        } else if (g_work_mode == WORK_MODE_2_GYRO) {
            gyro_init();
        }

        bluetooth_check_connection();
        
        // LED控制：严格按照设计文档
        if (g_bt_state == BT_STATE_CONNECTED) {
            led_set_on();      // 连接后常亮
        } else {
            led_set_blink_fast();  // 断开后快闪（唯一闪烁模式）
        }
        
        // 获取按键状态
        uint8_t new_key_state = button_get_state();
        
        // 数据发送逻辑
        uint32_t current_time = s_ms_ticks;
        if ((new_key_state != g_current_key_state) || 
            (g_work_mode == WORK_MODE_2_GYRO && 
             (current_time - g_last_packet_time) >= PACKET_SEND_INTERVAL_MS)) {
            
            g_current_key_state = new_key_state;
            g_last_packet_time = current_time;
            
            if (g_bt_state == BT_STATE_CONNECTED) {
                send_key_status_packet();
            }
        }
        
        led_update();
        
        // 检查休眠超时
        if (bluetooth_check_sleep_timeout()) {
            enter_sleep_mode();
        }
        
        delay_ms(1);
    }
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