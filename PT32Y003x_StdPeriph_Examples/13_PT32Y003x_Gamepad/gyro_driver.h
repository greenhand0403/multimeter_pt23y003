// gyro_driver.h
#ifndef GYRO_DRIVER_H
#define GYRO_DRIVER_H

#include "system_config.h"
#define MPU_ADDR        0xD0    // 0x68<<1 (AD0=0)；若AD0=1则用0xD2

#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B

#define REG_ACCEL_XOUT_H 0x3B   // 连续读到 0x48 可拿到 Acc/Temp/Gyro

extern void gyro_init(void);
extern void gyro_first_read(void);

#endif