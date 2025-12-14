#include "bluetooth_driver.h"
#include <string.h>
// uint32_t last_send_time = 0;

// extern volatile uint32_t s_ms_ticks;
extern void UART_SendString(UART_TypeDef* UARTx, const char *str);
extern uint8_t button_get_state(void);

/* 不要包含 <ctype.h>，使用轻量级替代以避免引入大块 libc */
static int isxdigit(int c)
{
    return ( (c >= '0' && c <= '9') ||
             (c >= 'A' && c <= 'F') ||
             (c >= 'a' && c <= 'f') );
}
static uint8_t hex_to_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0; // 或者你可以做错误处理
}

// ===== 发送原始数据（用于AT命令）=====
void bluetooth_send_raw_data(uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_SendData(UART0, data[i]);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TXE) == RESET);
    }
}

// ===== 发送协议数据包 =====
void bluetooth_send_packet(protocol_packet_t* packet)
{
    // TODO: 每20ms才发送一次
    // if (s_ms_ticks - last_send_time >= 20)
    // {
        uint8_t* buffer = (uint8_t*)packet;
        bluetooth_send_raw_data(buffer, sizeof(protocol_packet_t));
        // last_send_time = s_ms_ticks;
    // }
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
    // 优先级需比定时器高（此值应该小一点），否则会导致定时器中断先执行，导致蓝牙模块无法正常工作
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

// 启动蓝牙配置流程
void bluetooth_configure_name_start(void)
{
    if (g_bt_config_state == BT_CFG_STATE_IDLE)
    {
        g_bt_config_state = BT_CFG_STATE_QUERY_NAME;
        // 开始查询当前名称
        bluetooth_send_at_command("AT+TM\r\n");
    }
    // bluetooth_send_at_command("AT+CW\r\n"); // 蓝牙模块恢复出厂设置
}
// 处理传入的一行（不含 \r\n），例如 "OK"、"AT+BMONBOTS-1024"、"TB+CF7FA6F77DAE"
void ProcessBluetoothResponse(char* line)
{
    // 安全检查
    if (line == NULL || line[0] == '\0') return;

    UART_SendString(UART1, "RX0:");
    UART_SendString(UART1, line);
    UART_SendString(UART1, "\r\n");

    // 如果有模块在返回 OK，暂时忽略
    if (strcmp(line, "OK") == 0) {
        // 如果已经发送了设置名字，那么再发送复位命令
        if (g_bt_config_state == BT_CFG_STATE_SET_NAME)
        {
            g_bt_config_state = BT_CFG_STATE_COMPLETE;
        }
        else if (g_bt_config_state==BT_CFG_STATE_COMPLETE)
        {
            UART_SendString(UART1, "softreset ble\r\n");
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
            // 记录合法的 MAC 地址
            Legal_MAC[0] = (hex_to_val(name_suffix[0]) << 4) | hex_to_val(name_suffix[1]);
            Legal_MAC[1] = (hex_to_val(name_suffix[2]) << 4) | hex_to_val(name_suffix[3]);
            UART_SendString(UART1, "Legal_MAC:");
            UART_SendString(UART1, name_suffix);
            UART_SendString(UART1, "\r\n");
            if (strlen(name_suffix) >= 2) {
                BLE_NAME_LEGAL = 1;
            }
            return; // 已处理完该行，直接返回
        } else {
            // Debug_Printf("ilegal\r\n");
            // 名字不是期望格式，开始查询 MAC（并且**立即返回**，不要继续用当前行解析MAC）
            // if (g_bt_config_state == BT_CFG_STATE_IDLE || g_bt_config_state == BT_CFG_STATE_QUERY_NAME) {
                UART_SendString(UART1, "get mac\r\n");
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
            if (isxdigit((unsigned char)*hexp)) {
                const char* start = hexp;
                int cnt = 0;
                while (isxdigit((unsigned char)*hexp)) { cnt++; hexp++; }
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

                    Legal_MAC[0] = (hex_to_val(p[0]) << 4) | hex_to_val(p[1]);
                    Legal_MAC[1] = (hex_to_val(p[2]) << 4) | hex_to_val(p[3]);

                    p += 4;

                    p[0] = '\r';
                    p[1] = '\n';
                    p += 2;
                    *p = '\0';

                    UART_SendString(UART1, "setname:");
                    UART_SendString(UART1, new_name);
                    UART_SendString(UART1, "\r\n");
                    
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
// ===== 发送按键状态包 =====
void send_connect_packet(void)
{
    protocol_packet_t packet;
    
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_STATUS;
    
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
    if (button_get_state() == 0)
    {
        return;
    }
    
    protocol_packet_t packet;
    
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_STATUS;
    
    packet.data[0] = Legal_MAC[0];
    packet.data[1] = Legal_MAC[1];
    packet.data[2] = button_get_state();
    packet.data[3] = 0;
    packet.data[4] = 0;
    packet.data[5] = 0;
    
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
void bluetooth_send_first_connect_packet(void)
{
    protocol_packet_t packet;
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_CONNECT;
    packet.data[0] = Legal_MAC[0];
    packet.data[1] = Legal_MAC[1];
    packet.data[2] = 0;
    packet.data[3] = 0;
    packet.data[4] = 0;
    packet.data[5] = 0;
    packet.seq_num = g_seq_num++;
    // 检验和
    uint16_t crc = packet.cmd_type + (packet.data[0] + packet.data[1] + 
        packet.data[2] + packet.data[3] + packet.data[4] + 
        packet.data[5]) + packet.seq_num;
        
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low = (uint8_t)(crc & 0xFF);
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    bluetooth_send_packet(&packet);
}