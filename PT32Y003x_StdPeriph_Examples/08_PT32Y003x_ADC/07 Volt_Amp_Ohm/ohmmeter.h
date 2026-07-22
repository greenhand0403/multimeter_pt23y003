#ifndef OHMMETER_H
#define OHMMETER_H

#include <stdint.h>

/* 兆欧档开路ADC门限：实测开路约3633，留一定裕量 */
#define MOHM_OPEN_RAW          3500U

/* 兆欧档公式参数 */
#define MOHM_RAW_ZERO          55.0f
#define MOHM_RAW_FULL          3686.0f
#define MOHM_BASE_RESISTANCE   466000.0f

/*
 * 由开路ADC门限换算出来的最大可测电阻。
 * 约为8.63MΩ。
 */
#define MOHM_MAX_RESISTANCE    (MOHM_BASE_RESISTANCE * ((float)MOHM_OPEN_RAW - MOHM_RAW_ZERO) / (MOHM_RAW_FULL - (float)MOHM_OPEN_RAW))

typedef enum
{
    OHM_RANGE_OHM = 0,
    OHM_RANGE_KOHM,
    OHM_RANGE_MOHM
} ohmmeter_range_t;

typedef enum
{
    OHMMETER_SELECT_RANGE = 0,
    OHMMETER_MEASURE
} ohmmeter_state_t;

void Ohmmeter_Init(void);
void Ohmmeter_Update(void);

void Ohmmeter_GPIO_Init(void);
void Ohmmeter_AllOff(void);

float Ohmmeter_GetResistance(void);
float Ohmmeter_GetADCVoltage(void);
uint16_t Ohmmeter_GetRaw(void);

ohmmeter_range_t Ohmmeter_GetRange(void);
ohmmeter_state_t Ohmmeter_GetState(void);

#endif