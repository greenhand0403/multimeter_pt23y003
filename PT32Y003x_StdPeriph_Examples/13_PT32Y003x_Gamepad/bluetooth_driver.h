#include "system_config.h"

// 蓝牙配置状态机
typedef enum {
    BT_CFG_STATE_IDLE = 0,          // 空闲状态
    BT_CFG_STATE_QUERY_NAME,        // 已查询当前名称
    BT_CFG_STATE_QUERY_MAC,         // 已查询MAC地址
    BT_CFG_STATE_SET_NAME,          // 已设置新名称
    BT_CFG_STATE_COMPLETE           // 配置完成
} bt_config_state_t;

// 指令类别 01:手柄建立连接 02:手柄状态数据
volatile uint8_t g_current_key_state = 0;
volatile uint8_t g_seq_num = 0;  // 指令流水号
volatile uint32_t g_last_packet_time = 0;  // 上次发送包的时间戳，用于每20ms发送陀螺仪数据的 逻辑

volatile bluetooth_state_t g_bt_state = BT_STATE_DISCONNECTED;
volatile bt_config_state_t g_bt_config_state = BT_CFG_STATE_IDLE;
volatile uint8_t g_bt_config_complete = 0;
uint32_t last_activity_time = 0;
static uint8_t mac_address[6] = {0}; // 存储MAC地址

// 蓝牙名称检测
uint8_t Legal_Name = 0;
// 保存蓝牙地址后两字节
uint8_t Legal_MAC[2];

// 给串口重点关注的变量
uint16_t rx_buffer[64] = {0};
uint8_t rx_index = 0;
volatile uint8_t bluetooth_ready = 0;

void bluetooth_send_raw_data(uint8_t* data, uint16_t len);
void bluetooth_send_packet(protocol_packet_t* packet);
void bluetooth_send_at_command(const char* command);
void bluetooth_init(void);
void bluetooth_configure_name_start(void);  // 启动配置流程
void ProcessBluetoothResponse(const char* line);

// ===== 发送连接指令 =====
void bluetooth_send_first_connect_packet(void);

// bt_config_state_t get_bt_config_state(void);
// void bluetooth_check_connection(void);
// uint8_t bluetooth_check_sleep_timeout(void);