// 电压表状态
//     电压表标定参数
//     VoltTask_Init
//     VoltTask_Update
#include "voltmeter.h"
#include "meter_adc.h"
#include "uart.h"

#include "delay.h"

extern volatile uint32_t s_ms_ticks;

typedef struct
{
    uint32_t next_ms;
    // PA1 读到ADC并换算后的电压值
    float adc_voltage;
    // 电压表测量节点的电压
    float voltage;
} voltmeter_ctx_t;

static voltmeter_ctx_t s_volt;

void Voltmeter_Init(void)
{
    s_volt.next_ms = s_ms_ticks;
    s_volt.adc_voltage = 0.0f;
    s_volt.voltage = 0.0f;
}

void Voltmeter_Update(void)
{
    uint32_t now = s_ms_ticks;

    if ((int32_t)(now - s_volt.next_ms) < (int32_t)VOLT_SAMPLE_PERIOD_MS)
    {
        return;
    }

    s_volt.next_ms = now;

    uint16_t raw = MeterADC_ReadPA1(VOLT_AVG_SAMPLES);

    // 当前正常正向测量只使用0～4095范围；
    // 反接时会出现约8190的异常码，按无效输入归零
    // if (raw > 4095U)
    // {
    //     raw = 0U;
    // }

#if ENABLE_LOG
    LOGF("VOLT PA1 raw=%u\r\n", raw);
#endif
    float adc_v = MeterADC_RawToVoltage(raw);
    // 根据分压公式，将 PA1 读到的电压值转换为电压表测量节点的电压
    float measured = adc_v * K_VOLT_SLOPE;
    // 实测线性标定参数，用于误差补偿
    measured = measured * VOLT_CAL_GAIN + VOLT_CAL_OFFSET_V;
    // 只支持正向测量
    // 开路残留约0.10V，低于门限直接归零
    if (measured < VOLT_ZERO_DEADBAND_V)
    {
        measured = 0.0f;
    }
    // 上限钳位，防止异常数据影响测量
    if (measured > VOLT_MAX_V)
    {
        measured = VOLT_MAX_V;
    }

    s_volt.adc_voltage = adc_v;
    s_volt.voltage = measured;
}

float Voltmeter_GetVoltage(void)
{
    return s_volt.voltage;
}

float Voltmeter_GetADCVoltage(void)
{
    return s_volt.adc_voltage;
}