#include "bluetooth_driver.h"
#include <string.h>
// ===== 发送原始数据（用于AT命令）=====
void bluetooth_send_raw_data(uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_SendData(UART0, data[i]);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TXE) == RESET);
    }
    last_activity_time = s_ms_ticks;
}

// ===== 发送协议数据包 =====
void bluetooth_send_packet(protocol_packet_t* packet)
{
    uint8_t* buffer = (uint8_t*)packet;
    bluetooth_send_raw_data(buffer, sizeof(protocol_packet_t));
    
    if (g_bt_state == BT_STATE_DISCONNECTED) {
        g_bt_state = BT_STATE_CONNECTED;
    }
}

// ===== 发送AT命令 =====
void bluetooth_send_at_command(const char* command)
{
    bluetooth_send_raw_data((uint8_t*)command, strlen(command));
}

// ===== 初始化蓝牙模块 =====
void bluetooth_init(void)
{
    // UART0配置
    UART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0, ENABLE);
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0, ENABLE);
    
    uart.UART_BaudRate = BLUETOOTH_BAUD;
    uart.UART_WordLengthAndParity = UART_WordLengthAndParity_8D;
    uart.UART_StopBitLength = UART_StopBitLength_1;
    uart.UART_ParityMode = UART_ParityMode_Even;
    uart.UART_Receiver = UART_Receiver_Enable;
    uart.UART_LoopbackMode = UART_LoopbackMode_Disable;
    
    UART_Cmd(UART0, ENABLE);
    UART_Init(UART0, &uart);
    
    nvic.NVIC_IRQChannel = UART0_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x01;
    NVIC_Init(&nvic);
    UART_ITConfig(UART0, UART_IT_RXNEI, ENABLE);
    
    last_activity_time = s_ms_ticks;
    g_bt_state = BT_STATE_DISCONNECTED;
}

// ===== 配置蓝牙名称 =====
void bluetooth_configure_name(void)
{
    // 1. 查询当前名称
    bluetooth_send_at_command("AT+NAME?\r\n");
    delay_ms(100);
    
    // 2. 这里需要解析返回的名称，检查是否为ONBOTS开头
    // 3. 如果不是，查询MAC地址
    bluetooth_send_at_command("AT+MAC?\r\n");
    delay_ms(100);
    
    // 4. 解析MAC地址并设置新名称
    // 格式: AT+NAME=ONBOTS-XXYY\r\n
    // 5. 复位模块
    bluetooth_send_at_command("AT+RESET\r\n");
    delay_ms(1000); // 等待复位完成
}

// ===== 检查连接状态 =====
void bluetooth_check_connection(void)
{
    uint32_t current_time = s_ms_ticks;
    if ((current_time - last_activity_time) > 3000) {
        if (g_bt_state != BT_STATE_DISCONNECTED) {
            g_bt_state = BT_STATE_DISCONNECTED;
        }
    }
}

// ===== 检查休眠超时 =====
uint8_t bluetooth_check_sleep_timeout(void)
{
    uint32_t current_time = s_ms_ticks;
    if (g_bt_state == BT_STATE_DISCONNECTED) {
        if ((current_time - last_activity_time) >= AUTO_SLEEP_TIMEOUT_MS) {
            return 1;
        }
    }
    return 0;
}