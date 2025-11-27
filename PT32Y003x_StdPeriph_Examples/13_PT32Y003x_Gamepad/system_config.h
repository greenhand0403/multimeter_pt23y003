// system_config.h
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include "PT32Y003x.h"
#include "core_cm0.h"

// ===== UART 配置 =====
#define BLUETOOTH_BAUD 115200
#define DEBUG_BAUD     115200

// ===== 协议常量 =====
#define PROTOCOL_HEADER_H 0xAA
#define PROTOCOL_HEADER_L 0x55
#define PROTOCOL_TAIL_H   0x55
#define PROTOCOL_TAIL_L   0xAA

#define CMD_TYPE_CONNECT 0x01
#define CMD_TYPE_STATUS  0x02

#define PACKET_SEND_INTERVAL_MS 20

// ===== 按键定义 =====
#define KEY_PRESSED 0  // 因为上拉，按下为低电平

// GPIO 映射（可选）
#define BT_CONNECT_LED_PIN GPIOD, GPIO_Pin_3
#define LED_PIN            GPIOD, GPIO_Pin_4

// ===== 枚举类型 =====
typedef enum {
    BT_STATE_DISCONNECTED = 0,
    BT_STATE_CONNECTED
} bluetooth_state_t;

typedef enum {
    WORK_MODE_IDLE = 0,
    WORK_MODE_1_SERVO,
    WORK_MODE_2_GYRO
} work_mode_t;

// ===== 结构体（协议包）=====
typedef struct {
    uint8_t header_h;
    uint8_t header_l;
    uint8_t cmd_type;
    uint8_t data[6];
    uint8_t seq_num;
    uint8_t crc_high;
    uint8_t crc_low;
    uint8_t tail_h;
    uint8_t tail_l;
} protocol_packet_t;

// ===== 全局变量声明（extern）=====
extern volatile work_mode_t g_work_mode;
extern volatile work_mode_t g_work_mode_prev;
extern volatile uint8_t g_seq_num;
extern volatile uint32_t g_last_packet_time;

#endif // SYSTEM_CONFIG_H