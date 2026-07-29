#ifndef OHMMETER_H
#define OHMMETER_H

#include <stdint.h>

/* 1M中间电阻时，兆欧档开路ADC门限：实测开路约2600，留一定裕量 */
#define MOHM_OPEN_RAW          2550U

/* 欧姆档公式参数（100Ω采样电阻版本） */
#define OHM_SCALE       97.98f
#define OHM_RAW_ZERO   105.32f
#define OHM_RAW_OPEN 3983.59f
/* 千欧档公式参数 */
#define KOHM_SCALE       5117.2f
#define KOHM_RAW_ZERO      22.65f
#define KOHM_RAW_OPEN    3975.35f
/* 兆欧档公式参数 */
#define MOHM_SCALE       320100.0f
#define MOHM_RAW_ZERO        23.7f
#define MOHM_RAW_OPEN      2655.5f

/*
 * 由开路ADC门限换算出来的最大可测电阻。
 * 约为8.63MΩ。
 */
#define MOHM_MAX_RESISTANCE    (MOHM_SCALE * ((float)MOHM_OPEN_RAW - MOHM_RAW_ZERO) / (MOHM_RAW_OPEN - (float)MOHM_OPEN_RAW))

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