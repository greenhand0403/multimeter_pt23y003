#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "PT32Y003x.h"
#include "PT32Y003x_gpio.h"
#include "PT32Y003x_uart.h"
#include "PT32Y003x_nvic.h"
#include "PT32Y003x_i2c.h"
#include "PT32Y003x_tim.h"
#include "PT32Y003x_pwm.h"
// #include <stdio.h>
// #include <stdarg.h>
#include "delay.h"

// ===== 系统配置 =====
#define BLUETOOTH_BAUD    115200
#define DEBUG_BAUD        115200
#define PACKET_SEND_INTERVAL_MS  20  // 20ms发送间隔

// ===== 按键定义 =====
#define KEY_UP      GPIO_ReadDataBit(GPIOA, GPIO_Pin_1)   // PA1
#define KEY_LEFT    GPIO_ReadDataBit(GPIOA, GPIO_Pin_2)   // PA2
#define KEY_DOWN    GPIO_ReadDataBit(GPIOC, GPIO_Pin_3)   // PC3
#define KEY_RIGHT   GPIO_ReadDataBit(GPIOC, GPIO_Pin_4)   // PC4
#define KEY_A       GPIO_ReadDataBit(GPIOC, GPIO_Pin_5)   // PC5
#define KEY_B       GPIO_ReadDataBit(GPIOC, GPIO_Pin_6)   // PC6

// 按键有效电平：低电平有效
#define KEY_PRESSED 0

// ===== LED定义 =====
#define LED_PIN     GPIOD, GPIO_Pin_4    // 蓝色LED连接到PD4

// 蓝牙连接状态指示 PD3
#define BT_CONNECT_LED_PIN  GPIOD, GPIO_Pin_3

// ===== 时间配置 =====
#define AUTO_SLEEP_TIMEOUT_MS  120000  // 2分钟自动休眠

// ===== 模式检测 =====
#define MODE_DETECT_PIN   GPIOA, GPIO_Pin_3  // PA3: 悬空=模式1, 接地=模式2
#define MODE_1    1  // 按键与舵机模式
#define MODE_2    2  // 陀螺仪模式

// ===== 通信协议定义 =====
#define PROTOCOL_HEADER_H  0x55
#define PROTOCOL_HEADER_L  0xAA
#define PROTOCOL_TAIL_H    0xFF
#define PROTOCOL_TAIL_L    0xFF

// 指令类型
#define CMD_TYPE_CONNECT   0x01  // 连接指令
#define CMD_TYPE_STATUS    0x02  // 状态数据

// ===== 数据包结构 =====
typedef struct {
    uint8_t header_h;      // 头码高字节
    uint8_t header_l;      // 头码低字节
    uint8_t cmd_type;      // 指令类型
    uint8_t data[6];       // 指令数据
    uint8_t seq_num;       // 指令流水号
    uint8_t crc_high;      // CRC高字节
    uint8_t crc_low;       // CRC低字节
    uint8_t tail_h;        // 结束码高字节
    uint8_t tail_l;        // 结束码低字节
} protocol_packet_t;

// ===== 全局状态定义 =====
typedef enum {
    BT_STATE_DISCONNECTED = 0,
    BT_STATE_CONNECTED
} bluetooth_state_t;

typedef enum {
    WORK_MODE_IDLE = 0,
    WORK_MODE_1_SERVO,     // 模式1：舵机控制
    WORK_MODE_2_GYRO       // 模式2：陀螺仪控制
} work_mode_t;

// ===== 全局变量声明 =====
extern volatile work_mode_t g_work_mode;
// 蓝牙状态连接与否，在蓝牙驱动头文件定义和初始化
extern volatile bluetooth_state_t g_bt_state;

extern volatile uint8_t g_current_key_state;
extern volatile uint8_t g_seq_num;  // 指令流水号
extern volatile uint32_t g_last_packet_time;
extern uint8_t BLE_NAME_LEGAL;

// 指令类别 01:手柄建立连接 02:手柄状态数据
// volatile uint8_t g_current_key_state = 0;
// volatile uint8_t g_seq_num = 0;  // 指令流水号
// volatile uint32_t g_last_packet_time = 0;  // 上次发送包的时间戳，用于每20ms发送陀螺仪数据的 逻辑

// 蓝牙名称是否合法 1 则合法
// uint8_t BLE_NAME_LEGAL = 0;

#endif // SYSTEM_CONFIG_H