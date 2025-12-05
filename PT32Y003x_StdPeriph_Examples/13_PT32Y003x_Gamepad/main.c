#include "system_config.h"
#include "delay.h"

// #include "pwm_driver.h"
// #include "servo_driver.h"
// 蓝牙名称是否合法 1 则合法
uint8_t BLE_NAME_LEGAL = 0;
extern uint8_t Legal_MAC[2];

// 记录上次活动时间，用于判断是否120秒未连接超时
extern uint32_t last_activity_time;
extern volatile uint32_t g_last_packet_time;  // 上次发送包的时间戳，用于每20ms发送陀螺仪数据的 逻辑
// 手柄当前的工作模式，影响手柄数据包代表的含义
// 模式一代表按键舵机模式，当触发按键中断时记录发送标志位，发送完按键数据后清除标志位
// 模式二代表陀螺仪模式，当触发定时器中断 (每20ms) 时记录发送标志位，发送完陀螺仪数据后清除标志位
volatile work_mode_t g_work_mode = WORK_MODE_IDLE;
volatile work_mode_t g_work_mode_prev = WORK_MODE_IDLE;

// 复制串口接收区用的临时行缓冲
#define LINE_BUF_SIZE 60
char line_buf[LINE_BUF_SIZE];
volatile uint16_t line_len = 0;

extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;
extern volatile uint8_t rx_ring[LINE_BUF_SIZE];

// volatile uint16_t g_pwm_duty = 0;
// volatile uint8_t g_servo_angle = 90; // 默认90度

// 逐字节发送缓冲区
// uint16_t rx_buffer[64] = {0};
// uint8_t rx_index = 0;

// ===== 外部函数声明 =====
extern void led_init(void);
extern void led_set_blink_fast(void);  // 只保留快闪
extern void led_set_on(void);
extern void led_set_off(void);
extern void led_update(void);

extern void button_init(void);

extern void bluetooth_init(void);

extern void bluetooth_configure_name_start(void);
// ===== 发送首个手柄上线数据包 =====
extern void bluetooth_send_first_connect_packet(void);
// ===== 发送按键状态数据包 =====
extern void send_key_status_packet(void);

// ===== 处理蓝牙响应 =====
extern void ProcessBluetoothResponse(char *line);

extern void pwm_init(void);
extern void servo_init(void);

extern void pwm_set_duty(uint16_t duty);
extern void servo_set_angle(uint8_t angle);

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
    
    if (GPIO_ReadDataBit(MODE_DETECT_PIN) == 1) {
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
    // button_init();
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
// 清除标志位
static void Wake_Key_EXITDisable(void)
{
    EXTI_ITConfig(EXTIA, GPIO_Pin_1, DISABLE);
    EXTI_ITConfig(EXTIA, GPIO_Pin_2, DISABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_3, DISABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_4, DISABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_5, DISABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_6, DISABLE);
    EXTI_ClearFlag(EXTIA, GPIO_Pin_1);
    EXTI_ClearFlag(EXTIA, GPIO_Pin_2);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_3);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_4);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_5);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_6);
    NVIC_DisableIRQ(EXTIA_IRQn);
    NVIC_DisableIRQ(EXTIC_IRQn);
}
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
    
    NVIC_InitTypeDef nvic, nvic2;
    nvic.NVIC_IRQChannel = EXTIA_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x00;
    NVIC_Init(&nvic);
    
    nvic2.NVIC_IRQChannel = EXTIC_IRQn;
    nvic2.NVIC_IRQChannelCmd = ENABLE;
    nvic2.NVIC_IRQChannelPriority = 0x01;
    NVIC_Init(&nvic2);
    
    PWR_EnterDeepSleepMode(PWR_DeepSleepEntry_WFI);

    // 唤醒后，清除标志位
    Wake_Key_EXITDisable();

    system_init();
    last_activity_time = s_ms_ticks;
    UART_SendString(UART1, "WAKE UP\r\n");
}
uint8_t count = 0;
// uint8_t hit_head = 0;
// uint8_t hit_tail = 0;
// uint8_t receive_buffer[10] = {0x55, 0xAA, 0x01, 0x14, 0x7F, 0x01, 0x00, 0x95, 0xFF, 0xFF};
void PollAndProcessUARTLines(void)
{
    uint16_t tmp = line_len;
    // 从环形缓冲读取字节，拼成行
    while (rx_tail != rx_head) {
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % LINE_BUF_SIZE;

        // 限制行长度，防止越界
        if (line_len < LINE_BUF_SIZE - 1) {
            line_buf[line_len++] = (char)b;
            // rx_buffer[rx_index++] = b;
        } else {
            // 行太长，丢弃并重置
            line_len = 0;
        }

        // 测试，直接透传 等待 UART1 发送缓冲区为空
        // while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
        // 直接发送原始字节
        // UART_SendData(UART1, b);

        // 检测到 \r\n 结尾（常见的是 \r\n 两字节）&& line_len >= 2 && line_buf[line_len - 2] == '\r'
        // 很神奇，这里面多一个判断条件 例如 g_work_mode == WORK_MODE_IDLE && 会导致后续的10字节数据包接收出错
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
        // 判断数据包头是否符合协议格式 55 AA 01 14 7F 01 00 95 FF FF
        // 表示PB4 输出PWM信号(1kHz)，总共255，所以7F代表占空比50%，95是校验和01+14+7F+01
        // 判断数据包头是否符合协议格式 55 AA 02 00 5A 02 00 5E FF FF
        // 表示PB5 SG90 舵机驱动信号将角度设置为90°，总共是(0°~180°)，所以5A代表90°，5E是校验和02+00+5A+02
        else if (g_work_mode == WORK_MODE_1_SERVO && line_buf[line_len-1] == 0xff && line_buf[line_len-2] == 0xff)
        {
            // ProcessBluetoothResponse(line_buf); // 建议把行内容传给处理函数
            // for (int j = line_len - 1; j >= 0; j--)
            // {
            //     UART_SendData(UART1, line_buf[rx_head - j]);
            //     while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
            // }
            // // 测试索引值
            // // rx_tail = rx_head;
            // line_len = 0;
            
            // __disable_irq();

            // for (int j = 1; j <= 10; j++)
            {
                // uint8_t i = (rx_head - line_len + j ) % LINE_BUF_SIZE;
                // UART_SendData(UART1, line_buf[i]);
                // while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
            }
            // 问题三，3次发送10字节包，分别打印0C 16 20是什么原因？
            UART_SendData(UART1, rx_head);
            while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
            UART_SendData(UART1, rx_tail);
            while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
            // 问题二编译报错
            // UART_SendData(UART1, line_len);
            // while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);

            // 测试索引值
            // rx_tail = rx_head;
            // line_len = 0;
            // __enable_irq();

            // 问题一 测试通过 可用
            // for (int j = 0; j < 10; j++)
            // {
            //     UART_SendData(UART1, line_buf[j+10*count]);
            //     while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
            // }
            // count = (count + 1) % (LINE_BUF_SIZE/10);
        }
    
        
    }
    
}
// 新增函数：尝试从环形缓冲区中解析一个完整的协议包
void TryParseProtocolPacket(void)
{
    // 简单的状态机解析 解析协议包
    uint8_t parse_state = 0;
    uint8_t packet_buffer[10] = {0};
    uint8_t packet_index = 0;

    while (rx_tail != rx_head) {
        uint8_t byte = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % 128;

        switch (parse_state) {
            case 0: // 寻找包头
                if (byte == PROTOCOL_HEADER_H) {
                    parse_state = 1;
                    packet_buffer[0] = byte;
                    packet_index = 1;
                }
                break;
            case 1: // 检查第二个包头字节
                if (byte == PROTOCOL_HEADER_L) {
                    packet_buffer[1] = byte;
                    parse_state = 2;
                    packet_index = 2;
                } else {
                    parse_state = 0; // 复位
                }
                break;
            default:
                packet_buffer[packet_index] = byte;
                packet_index++;
                if (packet_index == 10) {
                    // 尝试校验包尾
                    // if (packet_buffer[8] == PROTOCOL_TAIL_H && packet_buffer[9] == PROTOCOL_TAIL_L) {
                        // 包完整，进行处理
                        // TODO: 这里可以加CRC校验

                        // 根据指令类型处理
                        // if (packet_buffer[3] == 0x01) {
                            // PWM 控制 默认控制PB4
                            // g_pwm_duty = packet_buffer[4] * 4; // 占空比值 (0-1000)
                            // pwm_set_duty(packet_buffer[4] * 4);
                        // } else if (packet_buffer[3] == 0x02) {
                            // 舵机控制 默认控制PB5
                            // g_servo_angle = packet_buffer[4]; // 角度 (0-180)
                        //     if (g_servo_angle > 180) g_servo_angle = 180;
                        //     servo_set_angle(g_servo_angle);

                        // }
                    // }
                    // 无论校验成功与否，都复位状态机
                    parse_state = 0;
                    packet_index = 0;
                }
                break;
        }
    }
}
// ===== 主函数 =====
int main(void)
{
    uint16_t i = 0;

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
    // 记录离线时间
    last_activity_time = s_ms_ticks;
    while (1)
    {
        // 判断主机连接手柄
        if (GPIO_ReadDataBit(BT_CONNECT_LED_PIN) == 1) 
        {
            // 主机刚开始连接上，未发送过连接包
            if (g_bt_state == BT_STATE_DISCONNECTED) {
                led_set_on();
                g_bt_state = BT_STATE_CONNECTED;
                // for (int k = 0; k < 2; k++)
                {
                    delay_ms(500);
                    // 发送第一条上线消息
                    bluetooth_send_first_connect_packet();
                    // delay_ms(200);
                }
                UART_SendString(UART1, "FIRST CONNECT\r\n");
                g_work_mode_prev = WORK_MODE_IDLE;
            }
            // 已连接，进行消息发送和处理
            else
            {
                // 用户可能连接手柄后主动切换工作模式，所有需要每次都判断工作模式
                // 已经有延迟20ms了
                g_work_mode = detect_work_mode();
                // 首次进入工作模式或者检测到工作模式切换，需要初始化
                if (g_work_mode_prev==WORK_MODE_IDLE)
                {
                    if (g_work_mode == WORK_MODE_1_SERVO) {
                        // 初始化按键
                        button_init();
                        // 初始化 PWM 和舵机驱动
                        pwm_init();
                        servo_init();
                    } else if (g_work_mode == WORK_MODE_2_GYRO) {
                        // 初始化陀螺仪I2C接口
                        // gyro_init();
                    }
                    g_work_mode_prev = g_work_mode;
                }
                // 工作模式已初始化，继续处理后续业务逻辑
                if (g_work_mode == g_work_mode_prev)
                {
                    if (g_work_mode == WORK_MODE_1_SERVO) {
                        // 扫描按键状态，填到数据包里面
                        // 每20ms发送一次按键舵机数据包
                        if (s_ms_ticks - g_last_packet_time >= PACKET_SEND_INTERVAL_MS) {
                            send_key_status_packet();
                            g_last_packet_time = s_ms_ticks;
                        }

                        // TODO: 蓝牙接收器
                        // 从环形缓冲读取字节，拼成行
                        PollAndProcessUARTLines();

                        // 接收蓝牙模块发送到串口0的数据，转发到串口1调试
                        // while(rx_index)
                        // {
                        //     UART_SendData(UART1,rx_buffer[i++]);
                        //     if(i==rx_index)
                        //     {
                        //         i=0;
                        //         rx_index=0;
                        //     }
                        // }
                        
                        // 新增：尝试解析二进制控制指令包
                        // TryParseProtocolPacket();

                    } else if (g_work_mode == WORK_MODE_2_GYRO) {
                        // 请求陀螺仪数据，填到数据包里
                        // request_gyro_data();
                        // 每20ms发送一次陀螺仪数据包
                        if (s_ms_ticks - g_last_packet_time >= PACKET_SEND_INTERVAL_MS) {
                            // send_gyro_data_packet();
                            g_last_packet_time = s_ms_ticks;
                        }
                    }
                }
                else
                {
                    g_work_mode_prev = WORK_MODE_IDLE;
                }
            }
        }
        // 未连接手柄
        else
        {
            // 连接后主机断开蓝牙手柄的情况
            if (g_bt_state == BT_STATE_CONNECTED) {
                led_set_blink_fast();
                g_bt_state = BT_STATE_DISCONNECTED;
                // 刷新主机掉线时间戳
                last_activity_time = s_ms_ticks;
            }
            // 如果未连接持续120秒，则进入休眠模式
            if (s_ms_ticks - last_activity_time >= AUTO_SLEEP_TIMEOUT_MS)
            {
                UART_SendString(UART1, "SLEEP\r\n");
                enter_sleep_mode();
            }
        }
        led_update();
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