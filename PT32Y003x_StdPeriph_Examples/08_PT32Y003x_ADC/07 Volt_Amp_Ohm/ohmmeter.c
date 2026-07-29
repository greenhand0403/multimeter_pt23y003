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

#define KOHM_TO_OHM_RAW        400U
#define KOHM_TO_MOHM_RAW       3630U
#define OHM_TO_SELECT_RAW      3370U    //欧姆档去到千欧档选档
#define MOHM_TO_SELECT_RAW     420U     //兆欧档去到千姆档选档 

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
    float r;
    float x = (float)raw;

    // 短路
    if(raw <= 120U)
    {
        return 0.0f;
    }

    /*
     * 低阻区：约0～10Ω。
     * 10Ω实测raw约513。
     */
    if (raw <= 520U)
    {
        r = -0.0000138f*x*x +0.0364f*x -3.54f;

        if (r < 0.0f)
        {
            r = 0.0f;
        }

        return r;
    }

    /*
     * 10～500Ω使用分压型拟合公式。
     */
    if (x >= OHM_RAW_OPEN)
    {
        return MOHM_MAX_RESISTANCE;
    }

    return OHM_SCALE * (x - OHM_RAW_ZERO) / (OHM_RAW_OPEN - x);
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
    s_ohm.range = OHM_RANGE_OHM;
#ifdef ENABLE_LOG
    s_ohm.state = OHMMETER_MEASURE;
#else
    s_ohm.state = OHMMETER_SELECT_RANGE;
#endif
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
        LOGF("OHM range=%d PA1 raw=%u\r\n", s_ohm.range, s_ohm.raw);
#endif
        // 用这个公式的话就是锁死 3V 参考电压去换算
        s_ohm.adc_voltage = MeterADC_RawToVoltage(s_ohm.raw);

        float rx = compute_rx_by_range(s_ohm.range, s_ohm.raw);
#if ENABLE_LOG
#else
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
#endif
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
硬件又改了一版，这几项需要重新测试其原始的ADC值，请你根据测试结果调整代码参数，为了避免反复切换的档位，我认为换挡阈值不能留太大的余留空间。本身ADC的读数差异基本上不会超过5个单位
修改代码时，换挡逻辑也优化一下吧，500Ω及以下用欧姆档，500Ω~50kΩ含50k用千欧档，超过50k用兆欧档

测试结果如下，都是固定档位去测试并且禁用了换挡功能：

开机A板电压表ADC=27，电流表ADC=16，欧姆表欧姆档ADC=3987
开机A板电压表ADC=30，电流表ADC=8189，欧姆表欧姆档ADC=3990

数据格式：
待测电阻  A板ADC原始值 B板ADC原始值

欧姆档
短路=113=120
1Ω=136=143
5Ω=250=259
10Ω=502=513
50Ω=1410=1412
100Ω=2073=2074
200Ω=2705
300Ω=3032
500Ω=3346=3348
600Ω=3440=3442
开路=3987=3990

千欧档
短路=76=79
500Ω=376=374
600Ω=437=434
1k=670=658
2k=1137=1137
5.1k=1989=1989
10k=2630=2630
30k=3397=3397
50k=3604=3610
60k=3660=3667
开路=3971=3972

兆欧档
短路=75=79
50k=394=393
60k=450=450
100k=679=676
300k=1296=1295
500k=1626=1624
1M=2019=2016
3M=2403=2398
5M=2497=2495
开路=2657=2653

*/