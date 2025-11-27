// protocol.c
#include "protocol.h"
#include "bluetooth_driver.h" // 用于发送

volatile uint8_t g_seq_num = 0;
volatile uint32_t g_last_packet_time = 0;

void send_connect_packet(void)
{
    protocol_packet_t packet = {0};
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_CONNECT;
    
    // 使用蓝牙模块获取的合法 MAC
    packet.data[0] = Legal_MAC[0];
    packet.data[1] = Legal_MAC[1];
    packet.data[2] = 0;
    packet.data[3] = 0;
    packet.data[4] = 0;
    packet.data[5] = 0;
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + 
                   (packet.data[0] + packet.data[1] + packet.data[2] +
                    packet.data[3] + packet.data[4] + packet.data[5]) +
                   packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low  = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
}

void send_key_status_packet(uint8_t key_state)
{
    protocol_packet_t packet = {0};
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_STATUS;
    
    packet.data[0] = key_state;
    packet.data[1] = 0;
    packet.data[2] = 0;
    packet.data[3] = 0x5A; // 占位
    packet.data[4] = 0x5A;
    packet.data[5] = 0x5A;
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + 
                   (packet.data[0] + packet.data[1] + packet.data[2] +
                    packet.data[3] + packet.data[4] + packet.data[5]) +
                   packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low  = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
}

void send_gyro_data_packet(int16_t gx, int16_t gy, int16_t gz)
{
    // TODO: 实际打包陀螺仪数据（高8位+低8位）
    protocol_packet_t packet = {0};
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_STATUS;
    
    // 示例：data[0:1]=gx, [2:3]=gy, [4:5]=gz
    packet.data[0] = (uint8_t)(gx >> 8);
    packet.data[1] = (uint8_t)(gx & 0xFF);
    packet.data[2] = (uint8_t)(gy >> 8);
    packet.data[3] = (uint8_t)(gy & 0xFF);
    packet.data[4] = (uint8_t)(gz >> 8);
    packet.data[5] = (uint8_t)(gz & 0xFF);
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + 
                   (packet.data[0] + packet.data[1] + packet.data[2] +
                    packet.data[3] + packet.data[4] + packet.data[5]) +
                   packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low  = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
}