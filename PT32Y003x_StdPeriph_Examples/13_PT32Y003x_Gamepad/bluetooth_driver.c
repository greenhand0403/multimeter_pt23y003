// bluetooth_driver.c
#include "bluetooth_driver.h"
#include <string.h>
#include <PT32Y003x_gpio.h>
#include <PT32Y003x_uart.h>
#include <PT32Y003x_nvic.h>
#include "protocol.h"

extern void Debug_Printf(const char *format, ...);
extern void led_set_on(void);

// ===== 内部辅助函数 =====
static inline int my_isxdigit(int c) {
    return ((c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'F') ||
            (c >= 'a' && c <= 'f'));
}

static inline int hex_char_to_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// ===== 蓝牙专属全局变量（定义）=====
volatile bluetooth_state_t g_bt_state = BT_STATE_DISCONNECTED;
volatile bt_config_state_t g_bt_config_state = BT_CFG_STATE_IDLE;
uint8_t BLE_NAME_LEGAL = 0;
uint8_t Legal_MAC[2] = {0x8F, 0x2E}; // 默认值

// ===== 串口接收环形缓冲（内部 static）=====
#define RX_RING_SIZE 128
static volatile uint8_t rx_ring[RX_RING_SIZE] = {0};
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint8_t g_rx_line_complete = 0;

// ===== 对外提供读取行的函数（可选）=====
// 此处简化：直接在中断中调用 ProcessBluetoothResponse

// ===== 发送函数 =====
void bluetooth_send_raw_data(uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_SendData(UART0, data[i]);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TXE) == RESET);
    }
}

void bluetooth_send_packet(protocol_packet_t* packet)
{
    bluetooth_send_raw_data((uint8_t*)packet, sizeof(protocol_packet_t));
}

void bluetooth_send_at_command(const char* command)
{
    bluetooth_send_raw_data((uint8_t*)command, strlen(command));
}

// ===== 初始化 =====
void bluetooth_init(void)
{
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0, ENABLE);
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0, ENABLE);

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

// ===== 配置流程 =====
void bluetooth_configure_name_start(void)
{
    if (g_bt_config_state == BT_CFG_STATE_IDLE)
    {
        g_bt_config_state = BT_CFG_STATE_QUERY_NAME;
        bluetooth_send_at_command("AT+TM\r\n");
    }
}

void ProcessBluetoothResponse(const char* line)
{
    if (!line || !line[0]) return;
    Debug_Printf("RX0:%s\r\n", line);

    if (strcmp(line, "OK") == 0) {
        if (g_bt_config_state == BT_CFG_STATE_SET_NAME) {
            g_bt_config_state = BT_CFG_STATE_COMPLETE;
        } else if (g_bt_config_state == BT_CFG_STATE_COMPLETE) {
            Debug_Printf("softreset ble\r\n");
            bluetooth_send_at_command("AT+CZ\r\n");
        }
        return;
    }

    if (strstr(line, "TM+") != NULL) {
        if (strstr(line, "ONBOTS-")) {
            BLE_NAME_LEGAL = 1;
        } else {
            Debug_Printf("get mac\r\n");
            g_bt_config_state = BT_CFG_STATE_QUERY_MAC;
            bluetooth_send_at_command("AT+TN\r\n");
            return;
        }
    }

    if (strstr(line, "TB+") != NULL && g_bt_config_state == BT_CFG_STATE_QUERY_MAC) {
        const char* hexp = line;
        while (*hexp) {
            if (my_isxdigit((unsigned char)*hexp)) {
                const char* start = hexp;
                int cnt = 0;
                while (my_isxdigit((unsigned char)*hexp)) { cnt++; hexp++; }
                if (cnt >= 4) {
                    char new_name[32];
                    snprintf(new_name, sizeof(new_name),
                             "AT+BMONBOTS-%c%c%c%c\r\n",
                             *(start+2), *(start+3), *(start+0), *(start+1));

                    Legal_MAC[0] = (*(start+2) << 8) | *(start+3);
                    Legal_MAC[1] = (*(start+0) << 8) | *(start+1);

                    Debug_Printf("setname:%s,mac:%04x%04x\r\n", 
                                 new_name, Legal_MAC[0], Legal_MAC[1]);

                    bluetooth_send_at_command(new_name);
                    g_bt_config_state = BT_CFG_STATE_SET_NAME;
                    return;
                }
            } else {
                hexp++;
            }
        }
    }
}

void bluetooth_send_first_connect_packet(void)
{
    send_connect_packet(); // 复用 protocol.c 的函数
}