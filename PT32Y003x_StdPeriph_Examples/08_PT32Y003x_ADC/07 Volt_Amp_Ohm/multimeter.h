#ifndef MULTIMETER_H
#define MULTIMETER_H

typedef enum
{
    METER_MODE_VOLT = 0,
    METER_MODE_AMP,
    METER_MODE_OHM
} meter_mode_t;

void Multimeter_Init(void);
void Multimeter_SetMode(meter_mode_t mode);
void Multimeter_NextMode(void);
void Multimeter_Update(void);

meter_mode_t Multimeter_GetMode(void);

#endif