#include "system_config.h"

// 蓝牙配置状态机，用于程序逻辑，设置合法的蓝牙名称
typedef enum {
    BT_CFG_STATE_IDLE = 0,          // 空闲状态
    BT_CFG_STATE_QUERY_NAME,        // 已查询当前名称
    BT_CFG_STATE_QUERY_MAC,         // 已查询MAC地址
    BT_CFG_STATE_SET_NAME,          // 已设置新名称
    BT_CFG_STATE_COMPLETE           // 配置完成
} bt_config_state_t;

volatile uint8_t g_seq_num = 0;  // 指令流水号
volatile uint32_t g_last_packet_time = 0;  // 上次发送包的时间戳，用于每20ms发送陀螺仪数据的 逻辑

volatile bluetooth_state_t g_bt_state = BT_STATE_DISCONNECTED;
volatile bt_config_state_t g_bt_config_state = BT_CFG_STATE_IDLE;

uint32_t last_activity_time = 0; // 用于未连接蓝牙时120秒 自动休眠

// 保存蓝牙地址后两字节
uint8_t Legal_MAC[2];

void bluetooth_send_raw_data(uint8_t* data, uint16_t len);
void bluetooth_send_packet(protocol_packet_t* packet);
void bluetooth_send_at_command(const char* command);
void bluetooth_init(void);
void bluetooth_configure_name_start(void);  // 启动配置流程

void ProcessBluetoothResponse(char* line);

// ===== 发送连接指令 =====
void bluetooth_send_first_connect_packet(void);
void send_key_status_packet(void);