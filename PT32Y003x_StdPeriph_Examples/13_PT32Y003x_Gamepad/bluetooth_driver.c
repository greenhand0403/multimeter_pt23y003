#include "bluetooth_driver.h"
#include <string.h>

extern void Debug_Printf(const char *format, ...);
extern void led_set_on(void);
/* 不要包含 <ctype.h>，使用轻量级替代以避免引入大块 libc */
static inline int my_isxdigit(int c)
{
    return ( (c >= '0' && c <= '9') ||
             (c >= 'A' && c <= 'F') ||
             (c >= 'a' && c <= 'f') );
}

/* hex 字符转 0..15，若非法返回 -1 */
static inline int hex_char_to_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* 将连续的 4 个 hex 字符转换为两个字节（例如 "7DAE" -> b1=0x7D, b2=0xAE） 
   返回 0 成功，-1 失败（如果有非法字符） */
static inline int parse_4hex_to_bytes(const char *p, uint8_t *b1, uint8_t *b2)
{
    int n0 = hex_char_to_nibble(p[0]);
    int n1 = hex_char_to_nibble(p[1]);
    int n2 = hex_char_to_nibble(p[2]);
    int n3 = hex_char_to_nibble(p[3]);
    if (n0 < 0 || n1 < 0 || n2 < 0 || n3 < 0) return -1;
    *b1 = (uint8_t)((n0 << 4) | n1);
    *b2 = (uint8_t)((n2 << 4) | n3);
    return 0;
}

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
// 启动蓝牙配置流程
void bluetooth_configure_name_start(void)
{
    g_bt_config_state = BT_CFG_STATE_QUERY_NAME;
    g_bt_config_complete = 0;
    
    // 开始查询当前名称
    bluetooth_send_at_command("AT+TM\r\n");
}
// 获取当前配置状态（供外部查询）
bt_config_state_t get_bt_config_state(void)
{
    return g_bt_config_state;
}
// 传入一行（不含 \r\n），例如 "OK"、"AT+BMONBOTS-1024"、"TB+CF7FA6F77DAE"
void ProcessBluetoothResponse(const char* line)
{
    // 安全检查
    if (line == NULL || line[0] == '\0') return;

    Debug_Printf("BT LINE: %s\r\n", line);

    // 如果有模块在返回 OK，暂时忽略
    if (strcmp(line, "OK") == 0) {
        return;
    }

    // 检查是否包含 ONBOTS 名称（模块回复是 AT+BMONBOTS-xxxx 或 AT+BMKTB3B8A）
    const char* p = strstr(line, "ONBOTS-");
    if (p) {
        // p 指向 ONBOTS- 后面
        const char* suffix = p + strlen("ONBOTS-");
        // suffix 应该是 4 个字符（例如 "1024"），但也要容错
        char name_suffix[8] = {0};
        strncpy(name_suffix, suffix, 4); // 最多4个
        name_suffix[4] = '\0';
        Debug_Printf("Found name suffix: %s\r\n", name_suffix);

        // 这里可以判断格式合法后点亮 LED
        if (strlen(name_suffix) >= 2) {
            led_set_on();
            // 标记蓝牙名已正确
            // g_bt_state = BT_STATE_CONFIGURED; // 示例
        }
        return;
    }

    // 检查是否是 MAC 返回
    // 找到连续的十六进制串（长度12对应MAC）
    // 例如 line == "TB+CF7FA6F77DAE"
    {
        const char* hexp = line;
        while (*hexp) {
            if (my_isxdigit((unsigned char)*hexp)) {
                const char* start = hexp;
                int cnt = 0;
                while (my_isxdigit((unsigned char)*hexp)) { cnt++; hexp++; }
                if (cnt >= 4) {
                    const char* tail = start + cnt - 4;
                    uint8_t b1, b2;
                    if (parse_4hex_to_bytes(tail, &b1, &b2) == 0) {
                        char new_suffix[5];
                        /* 避免 snprintf，如果你已有非常小的串口打印函数（如 Bluetooth_Printf）
                        也要留意它是否会拉入 printf 的实现。为了最小化影响，我们用简单的字符构造： */
                        static const char hexchars[] = "0123456789ABCDEF";
                        new_suffix[0] = hexchars[(b1 >> 4) & 0xF];
                        new_suffix[1] = hexchars[b1 & 0xF];
                        new_suffix[2] = hexchars[(b2 >> 4) & 0xF];
                        new_suffix[3] = hexchars[b2 & 0xF];
                        new_suffix[4] = '\0';
                        
                        // char new_name[19];
                        // sprintf(new_name, "AT+BMONBOTS-%c%c%c%c\r\n", 
                        //     new_suffix[0], new_suffix[1], new_suffix[2], new_suffix[3]);
                        // Debug_Printf("New name: %s", new_name);
                        // bluetooth_send_at_command(new_name);
                        // delay_ms(100);
            
                        // // 5. 复位模块，正常会打印多行蓝牙模块信息
                        // bluetooth_send_at_command("AT+CZ\r\n");
                        // delay_ms(1000); // 等待复位完成
            
                        // // 6. 查询蓝牙名称
                        // bluetooth_configure_name_start();
                    }
                }
            } else {
                hexp++;
            }
        }
    }
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