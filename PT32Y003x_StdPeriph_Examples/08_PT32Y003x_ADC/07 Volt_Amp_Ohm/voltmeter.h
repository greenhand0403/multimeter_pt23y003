#ifndef VOLTMETER_H
#define VOLTMETER_H

#define VOLT_SAMPLE_PERIOD_MS  300U
#define VOLT_AVG_SAMPLES       5U

#define K_VOLT_SLOPE           4.4f
#define VOLT_CAL_GAIN          1.01f
#define VOLT_CAL_OFFSET_V     (-0.05f)

#define VOLT_ZERO_DEADBAND_V   0.10f
#define VOLT_MAX_V             12.0f

typedef struct
{
    float voltage;
    float adc_voltage;
} voltmeter_result_t;

void Voltmeter_Init(void);
void Voltmeter_Update(void);

float Voltmeter_GetVoltage(void);
float Voltmeter_GetADCVoltage(void);

#endif