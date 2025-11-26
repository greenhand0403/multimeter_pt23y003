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

    Debug_Printf("RX0:%s\r\n", line);

    // 如果有模块在返回 OK，暂时忽略
    if (strcmp(line, "OK") == 0) {
        // 如果已经发送了设置名字，那么再发送复位命令
        if (g_bt_config_state == BT_CFG_STATE_SET_NAME)
        {
            g_bt_config_state = BT_CFG_STATE_COMPLETE;
        }
        else if (g_bt_config_state==BT_CFG_STATE_COMPLETE)
        {
            Debug_Printf("softreset ble\r\n");
            bluetooth_send_at_command("AT+CZ\r\n");
        }
        return;
    }

    // if (g_bt_config_state == BT_CFG_STATE_COMPLETE && strstr(line, "QL+") != NULL)
    // {
        // 蓝牙模块默认发送初始信息，其中最后一个是 QL+00 表示正常工作模式中
        // 我们需要等待它返回QL+之后再发查询名字的命令？
    // }
    // 如果返回的是名称行，例如 "TM+OBTEST-1024" 或 "TM+ONBOTS-XXXX"
    // 既处理蓝牙名称查询指令的回复，也响应软复位后的蓝牙信息回复，从中检查蓝牙名称
    if (strstr(line, "TM+") != NULL) {
        // 解析名字行
        const char* p = strstr(line, "ONBOTS-");
        if (p) {
            // 名字已经是 ONBOTS-xxxx，直接标记合法
            const char* suffix = p + strlen("ONBOTS-");
            char name_suffix[8] = {0};
            strncpy(name_suffix, suffix, 4);
            name_suffix[4] = '\0';
            Debug_Printf("name_suffix:%s\r\n", name_suffix);
            if (strlen(name_suffix) >= 2) {
                BLE_NAME_LEGAL = 1;
            }
            return; // 已处理完该行，直接返回
        } else {
            // Debug_Printf("ilegal\r\n");
            // 名字不是期望格式，开始查询 MAC（并且**立即返回**，不要继续用当前行解析MAC）
            // if (g_bt_config_state == BT_CFG_STATE_IDLE || g_bt_config_state == BT_CFG_STATE_QUERY_NAME) {
                Debug_Printf("get mac\r\n");
                g_bt_config_state = BT_CFG_STATE_QUERY_MAC;
                bluetooth_send_at_command("AT+TN\r\n"); // 发送查询MAC命令
                return; // 关键：返回，避免下面把当前名字行当作MAC解析
            // }
        }
    }
    // 下面只处理真正的 MAC 行 —— 先做一个更严格的前缀检查，避免误判
    // 例如你的模块 MAC 行是 "TB+CF7FA6F77DAE"，所以我们检查是否包含 "TB+"
    if (strstr(line, "TB+") != NULL &&g_bt_config_state == BT_CFG_STATE_QUERY_MAC) {
        // 找到连续的 hex 串并取合适的 4 个字符（此处以取 run 的前 4 个并按你的要求换位为例）
        const char* hexp = line;
        while (*hexp) {
            if (my_isxdigit((unsigned char)*hexp)) {
                const char* start = hexp;
                int cnt = 0;
                while (my_isxdigit((unsigned char)*hexp)) { cnt++; hexp++; }
                if (cnt >= 4) {
                    // 这里选择使用 run 的前 4 个字符作为基准（如 CF7F -> 7FCF）
                    char new_name[20];
                    const char prefix[] = "AT+BMONBOTS-";
                    char *p = new_name;
                    memcpy(p, prefix, sizeof(prefix) - 1);
                    p += (sizeof(prefix) - 1);

                    /* 拷入 4 个后缀字符 CF7F 就变成了 ONBOTS-7FCF */
                    p[0] = *(start + 2); // 第3个字符
                    p[1] = *(start + 3); // 第4个字符
                    p[2] = *(start + 0); // 第1个字符
                    p[3] = *(start + 1); // 第2个字符
                    p += 4;

                    p[0] = '\r';
                    p[1] = '\n';
                    p += 2;
                    *p = '\0';

                    Debug_Printf("setname:%s", new_name);

                    // 发送设置名称命令
                    bluetooth_send_at_command(new_name);
                    // 发完蓝牙模块会返回OK ，我们需要等待它返回OK 后，再发送 reset 命令
                    g_bt_config_state = BT_CFG_STATE_SET_NAME;
                    return; // 完成该行处理后返回
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