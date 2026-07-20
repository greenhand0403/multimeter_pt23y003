#ifndef METER_ADC_H
#define METER_ADC_H

#include <stdint.h>

#define METER_ADC_MAX_RAW       4095U
#define METER_ADC_VREF          3.0f

void MeterADC_Init(void);

/* 读取一次经过平均后的12位ADC值 */
uint16_t MeterADC_ReadPA1(uint8_t samples);
uint16_t MeterADC_ReadPD2(uint8_t samples);
uint16_t MeterADC_ReadPC4(uint8_t samples);

/* ADC原始值转电压 */
float MeterADC_RawToVoltage(uint16_t raw);

#endif