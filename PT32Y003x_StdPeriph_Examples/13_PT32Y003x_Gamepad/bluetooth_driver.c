#include "bluetooth_driver.h"
#include <string.h>

// ===== 发送原始数据（用于AT命令）=====
void bluetooth_send_raw_data(uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_SendData(UART0, data[i]);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TXE) == RESET);
    }
    // 记录最近活动时间
    // last_activity_time = s_ms_ticks;
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
    
    nvic.NVIC_IRQChannel = UART0_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x01;
    NVIC_Init(&nvic);
    UART_ITConfig(UART0, UART_IT_RXNEI, ENABLE);

    uart.UART_BaudRate = BLUETOOTH_BAUD;
    uart.UART_WordLengthAndParity = UART_WordLengthAndParity_8D;
    uart.UART_StopBitLength = UART_StopBitLength_1;
    uart.UART_ParityMode = UART_ParityMode_Even;
    uart.UART_Receiver = UART_Receiver_Enable;
    uart.UART_LoopbackMode = UART_LoopbackMode_Disable;
    
    UART_Init(UART0, &uart);
    UART_Cmd(UART0, ENABLE);
    
    // last_activity_time = s_ms_ticks;
    // g_bt_state = BT_STATE_DISCONNECTED;
}

// ===== 配置蓝牙名称 =====
void bluetooth_configure_name(void)
{
    // 1. 查询当前名称，已确认能正常发送
    bluetooth_send_at_command("AT+TM\r\n");
    delay_ms(100);
    
}
void ProcessBluetoothResponse(void)
{
    // 2. 这里解析返回的名称，检查是否为ONBOTS开头
    if (!Legal_Name)
    {
        // 处理正确的名称回包
        if (rx_index >= 14 && 
            rx_buffer[0] == 'T' && 
            rx_buffer[1] == 'M' && 
            rx_buffer[2] == '+' && 
            rx_buffer[3] == 'O' && 
            rx_buffer[4] == 'N' && 
            rx_buffer[5] == 'B' && 
            rx_buffer[6] == 'O' &&
            rx_buffer[7] == 'T' && 
            rx_buffer[8] == 'S' &&
            rx_buffer[9] == '-')
        {
            Legal_Name = 1;
        }
        // 处理Mac地址回包
        else if (rx_index >= 15 && 
            rx_buffer[0] == 'T' && 
            rx_buffer[1] == 'N' && 
            rx_buffer[2] == '+')
        {
            // 记录蓝牙地址的后两个字节，例如存为3412
            Legal_MAC[0] = rx_buffer[5];
            Legal_MAC[1] = rx_buffer[6];
            Legal_MAC[2] = rx_buffer[3];
            Legal_MAC[3] = rx_buffer[4];
            // 4. 解析MAC地址并设置新名称 需要记录为蓝牙名称，然后发送
            // AT+BMONBOTS-3412\r\n
            // 设置蓝牙名称为“ONBOTS-3412”
            // 目前测试机时 TN+CF7FA6F77DAE，所以记录为 ONBOTS-A67F
            char new_name[18];
            sprintf(new_name, "ONBOTS-%s\r\n", Legal_MAC);
            bluetooth_send_at_command(new_name);
            delay_ms(100);

            // 5. 复位模块
            bluetooth_send_at_command("AT+CZ\r\n");
            delay_ms(1000); // 等待复位完成
        }
        // 处理默认蓝牙名称时，且未获取Mac地址，则获取Mac地址
        else if (Legal_MAC[0]==0)
        {
            // 3. 如果不是ONBOTS-XXXX名称且未获取Mac地址，则查询MAC地址。返回TN+12345678AABB\r\n BLE 的蓝牙地址：0xBB、0xAA、0x78、0x56、0x34、0x12
            bluetooth_send_at_command("AT+TN\r\n");
            delay_ms(100);
        }
        
    }
    // 正常收发


}
// 2分钟无连接休眠的逻辑应该在LED驱动处负责休眠
// ===== 检查连接状态 =====
// void bluetooth_check_connection(void)
// {
//     uint32_t current_time = s_ms_ticks;
//     // 3秒不发送数据则认为连接断开
//     if ((current_time - last_activity_time) > 3000) {
//         if (g_bt_state != BT_STATE_DISCONNECTED) {
//             g_bt_state = BT_STATE_DISCONNECTED;
//         }
//     }
// }

// ===== 检查休眠超时 =====
// uint8_t bluetooth_check_sleep_timeout(void)
// {
//     uint32_t current_time = s_ms_ticks;
//     if (g_bt_state == BT_STATE_DISCONNECTED) {
//         if ((current_time - last_activity_time) >= AUTO_SLEEP_TIMEOUT_MS) {
//             return 1;
//         }
//     }
//     return 0;
// }