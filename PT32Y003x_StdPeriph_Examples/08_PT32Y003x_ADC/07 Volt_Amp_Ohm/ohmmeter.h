#ifndef OHMMETER_H
#define OHMMETER_H

#include <stdint.h>

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