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

// ===== 发送原始数据（用于AT命令）=====
void bluetooth_send_raw_data(uint8_t* data, uint16_t len)
{
    // TODO: 间隔20ms发送数据
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
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0, ENABLE);
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0, ENABLE);

    // UART0配置
    UART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    
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
}

// ===== 配置蓝牙名称 =====
// 获取当前配置状态（供外部查询）
// bt_config_state_t get_bt_config_state(void)
// {
//     return g_bt_config_state;
// }
// 启动蓝牙配置流程
void bluetooth_configure_name_start(void)
{
    if (g_bt_config_state == BT_CFG_STATE_IDLE)
    {
        g_bt_config_state = BT_CFG_STATE_QUERY_NAME;
        // 开始查询当前名称
        bluetooth_send_at_command("AT+TM\r\n");
    }
}
// 处理传入的一行（不含 \r\n），例如 "OK"、"AT+BMONBOTS-1024"、"TB+CF7FA6F77DAE"
void ProcessBluetoothResponse(const char* line)
{
    // 安全检查
    if (line == NULL || line[0] == '\0') return;

    Debug_Printf("BTL:%s\r\n", line);

    // 如果有模块在返回 OK，暂时忽略
    if (strcmp(line, "OK") == 0) {
        return;
    }

    // 检查是否包含 ONBOTS 名称（发送查询命令后，蓝牙模块回复是 AT+BMONBOTS-xxxx 或 AT+BMKTB3B8A）
    const char* p = strstr(line, "ONBOTS-");
    if (p) {
        // p 指向 ONBOTS- 后面
        const char* suffix = p + strlen("ONBOTS-");
        // suffix 应该是 4 个字符（例如 "1024"），但也要容错
        char name_suffix[8] = {0};
        strncpy(name_suffix, suffix, 4); // 最多4个
        name_suffix[4] = '\0';
        Debug_Printf("BTNL:%s\r\n", name_suffix);

        // 这里可以判断格式合法后点亮 LED
        if (strlen(name_suffix) >= 2) {
            // 标记蓝牙名已正确
            // g_bt_config_state = BT_CFG_STATE_QUERY_NAME;
            BLE_NAME_LEGAL = 1;
        }
        return;
    }

    // 走到这里说明前面判断蓝牙名称不合法，那么发送Mac地址查询的命令给蓝牙模块
    if (g_bt_config_state == BT_CFG_STATE_QUERY_NAME)
    {
        g_bt_config_state = BT_CFG_STATE_QUERY_MAC;
        // 开始查询当前Mac地址
        bluetooth_send_at_command("AT+TN\r\n");
        return;
    }
    
    // 检查是否连续的十六进制串（长度12对应MAC）
    // 例如 line == "TB+CF7FA6F77DAE"
    if (g_bt_config_state == BT_CFG_STATE_QUERY_MAC)
    {
        const char* hexp = line;
        while (*hexp) {
            if (my_isxdigit((unsigned char)*hexp)) {
                const char* start = hexp;
                int cnt = 0;
                while (my_isxdigit((unsigned char)*hexp))
                { 
                    cnt++;
                    hexp++;
                }
                // 走到这一步，说明是连续的12字节 CF7FA6F77DAE
                if (cnt >= 4) {
                    // 取前4个字节排列一下，加上前缀就能发送命令设置蓝牙名称了

                    /* 构造 "AT+BMONBOTS-XXXX\r\n" （共 18 字符）+ 1 字节终止符 => 19 字节 */
                    char new_name[20]; // 19 bytes needed, 20 安全
                    const char prefix[] = "AT+BMONBOTS-";
                    char *p = new_name;

                    /* 把前缀拷进 new_name */
                    memcpy(p, prefix, sizeof(prefix) - 1); // 不拷贝末尾 '\0'
                    p += (sizeof(prefix) - 1);

                    /* 拷入 4 个后缀字符 CF7F 就变成了 ONBOTS-7FCF */
                    p[0] = *(start+2);
                    p[1] = *(start+3);
                    p[2] = *(start+0);
                    p[3] = *(start+1);
                    p += 4;

                    /* 加上 CR LF 和终止符 */
                    p[0] = '\r';
                    p[1] = '\n';
                    p += 2;
                    *p = '\0';

                    Debug_Printf("BTNL:%s\r\n", new_name);
                    bluetooth_send_at_command(new_name);
                    g_bt_config_state = BT_CFG_STATE_SET_NAME;
                }
            } else {
                hexp++;
            }
        }
    }
    // 如果已经发送了设置名字，那么再发送复位命令
    if (g_bt_config_state == BT_CFG_STATE_SET_NAME)
    {
        bluetooth_send_at_command("AT+CZ\r\n");
        g_bt_config_state = BT_CFG_STATE_COMPLETE;
        // 打印一下状态
        Debug_Printf("BTCFST:%d\r\n", g_bt_config_state);
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