// protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "system_config.h"

void send_connect_packet(void);
void send_key_status_packet(uint8_t key_state);
void send_gyro_data_packet(int16_t gx, int16_t gy, int16_t gz); // Ê¾Àý²ÎÊý

#endif