#include "system_config.h"

volatile bluetooth_state_t g_bt_state = BT_STATE_DISCONNECTED;
static uint32_t last_activity_time = 0;
static uint8_t mac_address[6] = {0}; // 存储MAC地址

// 给串口重点关注的变量
volatile uint8_t rx_buffer[64] = {0};
volatile uint16_t rx_index = 0;
volatile uint8_t bluetooth_ready = 0;

void bluetooth_send_raw_data(uint8_t* data, uint16_t len);
void bluetooth_send_packet(protocol_packet_t* packet);
void bluetooth_send_at_command(const char* command);
void bluetooth_init(void);
void bluetooth_configure_name(void);
void bluetooth_check_connection(void);
uint8_t bluetooth_check_sleep_timeout(void);