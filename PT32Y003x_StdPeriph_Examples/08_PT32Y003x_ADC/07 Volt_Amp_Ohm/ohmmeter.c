// 欧姆表状态机
//     自动换挡
//     三档计算公式
//     换挡GPIO控制
//     OhmTask_Init
//     OhmTask_Update
#include "ohmmeter.h"
#include "meter_adc.h"

#include "PT32Y003x.h"
#include <PT32Y003x_gpio.h>
#include "delay.h"
#include "uart.h"

#define OHM_SAMPLE_PERIOD_MS   300U
#define OHM_AVG_SAMPLES        5U

#define KOHM_TO_OHM_RAW        410U
#define KOHM_TO_MOHM_RAW       3680U
#define OHM_TO_SELECT_RAW      3700U    //欧姆档去到千欧档选档
#define MOHM_TO_SELECT_RAW     445U     //兆欧档去到千姆档选档 

#define MOHM_SEL_PIN_PORT      GPIOB
#define MOHM_SEL_PIN_NUM       GPIO_Pin_1

extern volatile uint32_t s_ms_ticks;
extern void LCD_DelayOneFrame(void);
typedef struct
{
    ohmmeter_state_t state;
    ohmmeter_range_t range;

    uint32_t next_ms;

    uint16_t raw;
    float adc_voltage;
    float resistance;
} ohmmeter_ctx_t;

static ohmmeter_ctx_t s_ohm;

static float Ohmmeter_ComputeOhm(uint16_t raw);
static float Ohmmeter_ComputeKOhm(uint16_t raw);
static float Ohmmeter_ComputeMOhm(uint16_t raw);

static void Ohmmeter_SetRangePins(ohmmeter_range_t range);

static float Ohmmeter_ComputeOhm(uint16_t raw)
{
    float rx;

    /*
     * 短路实测raw约195；
     * 10Ω已经约970，因此300以下可安全视为短路。
     */
    if (raw < 300U)
    {
        return 0.0f;
    }

    /*
     * Ω档高端超出有效范围，交给kΩ档处理。
     * 正常自动换挡通常会先于这里触发。
     */
    if (raw >= 3950U)
    {
        return MOHM_MAX_RESISTANCE;
    }

    /*
     * 低阻区：
     * 10Ω raw≈970
     * 33Ω raw≈1680
     * 50Ω raw≈2059
     *
     * 使用带ADC零点偏移的拟合公式。
     */
    if (raw < 2100U)
    {
        rx = OHM_SCALE * ((float)raw - OHM_RAW_ZERO) / (OHM_RAW_OPEN - (float)raw);

        if (rx < 0.0f)
        {
            rx = 0.0f;
        }

        return rx;
    }

    /*
     * 中高阻区：
     * 原分压公式乘0.965后，在50～600Ω范围更准确。
     */
    rx = 51.0f * (float)raw / (4029.0f - (float)raw);

    rx *= 0.965f;

    return rx;
}
static float Ohmmeter_ComputeKOhm(uint16_t raw)
{
    if (raw >= 4000U)
    {
        return MOHM_MAX_RESISTANCE;
    }

    if (raw <= 37U)
    {
        return 0.0f;
    }

    return KOHM_SCALE * ((float)raw - KOHM_RAW_ZERO) / (KOHM_RAW_OPEN - (float)raw);
}
static float Ohmmeter_ComputeMOhm(uint16_t raw)
{
    if ((float)raw <= MOHM_RAW_ZERO)
    {
        return 0.0f;
    }

    if ((float)raw >= MOHM_RAW_OPEN)
    {
        return MOHM_MAX_RESISTANCE;
    }

    return MOHM_SCALE * ((float)raw - MOHM_RAW_ZERO) / (MOHM_RAW_OPEN - (float)raw);
}
// 根据档位自动计算电阻值 
static inline float compute_rx_by_range(ohmmeter_range_t range, uint16_t raw)
{
    switch (range)
    {
        case OHM_RANGE_OHM:
            return Ohmmeter_ComputeOhm(raw);

        case OHM_RANGE_KOHM:
            return Ohmmeter_ComputeKOhm(raw);

        case OHM_RANGE_MOHM:
            return Ohmmeter_ComputeMOhm(raw);

        default:
            return 0.0f;
    }
}
static void Ohmmeter_SetRangePins(ohmmeter_range_t range)
{
    /* 先全部关断 */
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_SetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);

    switch (range)
    {
    case OHM_RANGE_OHM:
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        break;

    case OHM_RANGE_KOHM:
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);
        break;

    case OHM_RANGE_MOHM:
        GPIO_ResetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);
        break;

    default:
        break;
    }

    delay_ms(10);
}

void Ohmmeter_Init(void)
{
    s_ohm.range = OHM_RANGE_KOHM;
    s_ohm.state = OHMMETER_SELECT_RANGE;
    // s_ohm.state = OHMMETER_MEASURE;
    s_ohm.next_ms = s_ms_ticks;

    s_ohm.raw = 0;
    s_ohm.adc_voltage = 0.0f;
    s_ohm.resistance = MOHM_MAX_RESISTANCE;

    Ohmmeter_SetRangePins(s_ohm.range);
    // 重置LCD刷新的计时器，防止初始化时刷新屏幕
    // g_lcd_buf.last_update_ms = s_ohm.next_ms + OHM_SAMPLE_PERIOD_MS;
}

// 每次调用仅推进一步；无阻塞、无 while(1)
void Ohmmeter_Update(void)
{   
    // 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((uint32_t)(now - s_ohm.next_ms) < OHM_SAMPLE_PERIOD_MS) return;
    s_ohm.next_ms = now;

    // 档位选择
    if (s_ohm.state == OHMMETER_SELECT_RANGE) {
        if (s_ohm.range != OHM_RANGE_KOHM)
        {
            s_ohm.range = OHM_RANGE_KOHM;
            Ohmmeter_SetRangePins(s_ohm.range);
        }
        s_ohm.raw = MeterADC_ReadPA1(OHM_AVG_SAMPLES);
        // 用这个公式的话就是锁死 3V 参考电压去换算
        s_ohm.adc_voltage = MeterADC_RawToVoltage(s_ohm.raw);
        
        if (s_ohm.raw < KOHM_TO_OHM_RAW)
        {
            s_ohm.range = OHM_RANGE_OHM;
            Ohmmeter_SetRangePins(s_ohm.range);
            LCD_DelayOneFrame();
        }
        else if (s_ohm.raw > KOHM_TO_MOHM_RAW)
        {
            s_ohm.range = OHM_RANGE_MOHM;
            Ohmmeter_SetRangePins(s_ohm.range);
            LCD_DelayOneFrame();
        }

        s_ohm.state = OHMMETER_MEASURE;
        
    } else {
        s_ohm.raw = MeterADC_ReadPA1(OHM_AVG_SAMPLES);
#if ENABLE_LOG
        LOGF("OHM PA1 raw=%u\r\n", s_ohm.raw);
#endif
        // 用这个公式的话就是锁死 3V 参考电压去换算
        s_ohm.adc_voltage = MeterADC_RawToVoltage(s_ohm.raw);

        float rx = compute_rx_by_range(s_ohm.range, s_ohm.raw);
        // M欧档退回千欧档、欧姆档切到千欧档、千欧档切到兆欧和欧姆档的逻辑
        switch (s_ohm.range)
        {
            case OHM_RANGE_OHM:
                // 510Ω实测ADC约3669，超过3700重新选档
                if (s_ohm.raw > OHM_TO_SELECT_RAW)
                {
                    s_ohm.state = OHMMETER_SELECT_RANGE;
                }
                break;

            case OHM_RANGE_KOHM:
                if (s_ohm.raw < KOHM_TO_OHM_RAW ||
                    s_ohm.raw > KOHM_TO_MOHM_RAW)
                {
                    s_ohm.state = OHMMETER_SELECT_RANGE;
                }
                break;

            case OHM_RANGE_MOHM:
                // 51kΩ在M档约ADC437
                // 低于445说明应回kΩ档重新判断
                if (s_ohm.raw < MOHM_TO_SELECT_RAW)
                {
                    s_ohm.state = OHMMETER_SELECT_RANGE;
                }
                break;

            default:
                break;
        }

        if (s_ohm.state == OHMMETER_SELECT_RANGE)
        {
            // 换档时跳过中间状态的显示
            LCD_DelayOneFrame();
        }
        else
        {
            s_ohm.resistance = rx;
        }
    }
}
void Ohmmeter_GPIO_Init(void)
{
    GPIO_InitTypeDef gi;

    // PA2 控制 Ω档
    gi.GPIO_Mode = GPIO_Mode_OutPP;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    gi.GPIO_Pin  = GPIO_Pin_2;
    GPIO_Init(GPIOA, &gi);

    // PA3 控制 kΩ档
    gi.GPIO_Pin  = GPIO_Pin_3;
    GPIO_Init(GPIOA, &gi);

    // 控制 MΩ档
    gi.GPIO_Pin  = MOHM_SEL_PIN_NUM;
    GPIO_Init(MOHM_SEL_PIN_PORT, &gi);
}
void Ohmmeter_AllOff(void)
{
    // 高电平，关断所有 P-MOS
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_SetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);
}
float Ohmmeter_GetResistance(void)
{
    return s_ohm.resistance;
}
float Ohmmeter_GetADCVoltage(void)
{
    return s_ohm.adc_voltage;
}
uint16_t Ohmmeter_GetRaw(void)
{
    return s_ohm.raw;
}
ohmmeter_range_t Ohmmeter_GetRange(void)
{
    return s_ohm.range;
}
ohmmeter_state_t Ohmmeter_GetState(void)
{
    return s_ohm.state;
}
/*
欧姆档
实际值	公式值	误差
10Ω	10.13Ω	+1.34%
33Ω	31.52Ω	?4.47%
50Ω	49.13Ω	?1.74%
99Ω	100.97Ω	+1.99%
200Ω	205.59Ω	+2.80%
300Ω	304.99Ω	+1.66%
516Ω	512.96Ω	?0.59%
591Ω	582.72Ω	?1.40%

千欧档
实际值	公式值	误差
517Ω	517.8Ω	+0.16%
981Ω	976.8Ω	?0.43%
5.1kΩ	5.142kΩ	+0.82%
19.6kΩ	19.509kΩ	?0.46%
29.6kΩ	29.468kΩ	?0.45%
51.2kΩ	51.316kΩ	+0.23%
56.2kΩ	56.265kΩ	+0.12%

兆欧档
实际值	公式值	误差
51.0kΩ	50.82kΩ	?0.36%
58.3kΩ	58.27kΩ	?0.05%
68.8kΩ	68.73kΩ	?0.10%
97.9kΩ	98.93kΩ	+1.05%
200.7kΩ	200.30kΩ	?0.20%
1.01MΩ	1.002MΩ	?0.75%
2MΩ	2.017MΩ	+0.83%
4MΩ	3.951MΩ	?1.22%
5.01MΩ	5.047MΩ	+0.74%
*/