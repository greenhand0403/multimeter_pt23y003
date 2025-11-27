// bluetooth_driver.h
#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H

#include "system_config.h"

typedef enum {
    BT_CFG_STATE_IDLE = 0,
    BT_CFG_STATE_QUERY_NAME,
    BT_CFG_STATE_QUERY_MAC,
    BT_CFG_STATE_SET_NAME,
    BT_CFG_STATE_COMPLETE
} bt_config_state_t;

// ===== 全局变量声明 =====
extern volatile bluetooth_state_t g_bt_state;
extern volatile bt_config_state_t g_bt_config_state;
extern uint8_t BLE_NAME_LEGAL;
extern uint8_t Legal_MAC[2];

// ===== 函数接口 =====
void bluetooth_init(void);
void bluetooth_send_raw_data(uint8_t* data, uint16_t len);
void bluetooth_send_packet(protocol_packet_t* packet);
void bluetooth_send_at_command(const char* command);
void bluetooth_configure_name_start(void);
void ProcessBluetoothResponse(const char* line);
void bluetooth_send_first_connect_packet(void);

#endif