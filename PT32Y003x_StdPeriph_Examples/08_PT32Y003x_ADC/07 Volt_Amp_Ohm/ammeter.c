// 电流表状态
//     电流表标定参数
//     AmpTask_Init
//     AmpTask_Update#include "ammeter.h"
#include "meter_adc.h"
#include "ammeter.h"
#include "uart.h"

extern volatile uint32_t s_ms_ticks;

typedef struct
{
    uint32_t next_ms;
    // ADC 原始值
    uint16_t raw;
    // ADC 换算后的电压值
    float adc_voltage;
    // 测量端流经采样电阻的电流值
    float current;
} ammeter_ctx_t;

static ammeter_ctx_t s_amp;

void Ammeter_Init(void)
{
    s_amp.next_ms = s_ms_ticks;
    s_amp.raw = 0;
    s_amp.adc_voltage = 0.0f;
    s_amp.current = 0.0f;
}

void Ammeter_Update(void)
{
    uint32_t now = s_ms_ticks;

    if ((int32_t)(now - s_amp.next_ms) < (int32_t)AMP_SAMPLE_PERIOD_MS)
    {
        return;
    }

    s_amp.next_ms = now;

    uint16_t raw = MeterADC_ReadPD2(AMP_AVG_SAMPLES);
#if ENABLE_LOG
    LOGF("AMP PD2 raw=%u\r\n", raw);
#endif
    float adc_v = MeterADC_RawToVoltage(raw);

    s_amp.raw = raw;
    s_amp.adc_voltage = adc_v;
    // 开路时实测8190～8191 此时显示电流为零
    if (raw >= AMP_OPEN_RAW_TH)
    {
        s_amp.current = 0.0f;
        return;
    }
    // 实际电流达到约2.9A后，ADC停留在4050～4090
    // 统一按3A超量程处理
    if (raw >= AMP_FULLSCALE_RAW_TH)
    {
        s_amp.current = AMP_FULLSCALE_A;
        return;
    }
    // 理论计算：0.1Ω采样电阻，放大倍数约10.1
    float current = adc_v / (AMP_SHUNT_OHM * AMP_FRONTEND_GAIN);
    // 实测线性校准，根据207mA～2005mA实测数据校准
    current = current * AMP_CAL_GAIN + AMP_CAL_OFFSET_A;
    // 小电流及负值归零，单向正电流测量，低于零点死区统一显示0
    if (current < AMP_ZERO_DEADBAND_A)
    {
        current = 0.0f;
    }
    // 满量程限制，防止异常数据影响测量
    if (current > AMP_FULLSCALE_A)
    {
        current = AMP_FULLSCALE_A;
    }

    s_amp.current = current;
}

float Ammeter_GetCurrent(void)
{
    return s_amp.current;
}

float Ammeter_GetADCVoltage(void)
{
    return s_amp.adc_voltage;
}

uint16_t Ammeter_GetRaw(void)
{
    return s_amp.raw;
}