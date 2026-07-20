#ifndef AMMETER_H
#define AMMETER_H

#include <stdint.h>

#define AMP_SAMPLE_PERIOD_MS    300U
#define AMP_AVG_SAMPLES         5U

#define AMP_OPEN_RAW_TH         6000U
#define AMP_FULLSCALE_RAW_TH    4050U

#define AMP_FULLSCALE_A         3.0f
#define AMP_CAL_GAIN            0.9765f
#define AMP_CAL_OFFSET_A       (-0.00535f)
#define AMP_ZERO_DEADBAND_A     0.010f

#define AMP_SHUNT_OHM           0.1f
#define AMP_FRONTEND_GAIN       10.1f

void Ammeter_Init(void);
void Ammeter_Update(void);

float Ammeter_GetCurrent(void);
float Ammeter_GetADCVoltage(void);
uint16_t Ammeter_GetRaw(void);

#endif