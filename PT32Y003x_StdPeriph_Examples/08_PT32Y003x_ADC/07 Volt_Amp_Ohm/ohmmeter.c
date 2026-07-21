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

#define OHM_SAMPLE_PERIOD_MS   300U
#define OHM_AVG_SAMPLES        5U

#define OHM_OPEN_VALUE         51000000.0f

#define KOHM_TO_OHM_RAW        430U
#define KOHM_TO_MOHM_RAW       3850U
#define OHM_TO_SELECT_RAW      3700U
#define MOHM_TO_SELECT_RAW      600U

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
    // 实测短路 raw约206
    if (raw < 250U)
    {
        return 0.0f;
    }

    if (raw >= 3950U)
    {
        return OHM_OPEN_VALUE;
    }

    float rx = 51.0f * raw / (4029.0f - raw);
    // 10Ω附近单独修正
    if (rx < 10.0f)
    {
        // rx = rx * 0.291f + 3.69f;
    }
    else
    {
        // 50～500Ω目前整体偏高约3%～6%
        rx *= 0.965f;
    }

    return rx;
}
static float Ohmmeter_ComputeKOhm(uint16_t raw)
{
    if (raw >= 4000U)
    {
        return OHM_OPEN_VALUE;
    }

    if (raw <= 37U)
    {
        return 0.0f;
    }

    return 5086.3f * ((float)raw - 37.2f) / (4029.0f - (float)raw);
}
static float Ohmmeter_ComputeMOhm(uint16_t raw)
{
    if (raw >= 3500U)
    {
        return OHM_OPEN_VALUE;
    }

    if (raw <= 55U)
    {
        return 0.0f;
    }

    return 466000.0f * ((float)raw - 55.0f) / (3686.0f - (float)raw);
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
    s_ohm.next_ms = s_ms_ticks;

    s_ohm.raw = 0;
    s_ohm.adc_voltage = 0.0f;
    s_ohm.resistance = OHM_OPEN_VALUE;

    Ohmmeter_SetRangePins(OHM_RANGE_KOHM);
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
                // 51kΩ在M档约ADC420
                // 低于300说明应回kΩ档重新判断
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
