// ADC初始化
//     ADC扫描
//     读取PA1、PD2、PC4
//     ADC原始值转电压
#include "meter_adc.h"

#include "PT32Y003x.h"
#include <PT32Y003x_gpio.h>
#include <PT32Y003x_adc.h>
#include "uart.h"

#define ADC_CH_VOLT_OHM  ADC_Channel_1
#define ADC_CH_AMP       ADC_Channel_6
#define ADC_CH_BATT      ADC_Channel_7

static uint16_t s_adc_pa1_raw;
static uint16_t s_adc_pd2_raw;
static uint16_t s_adc_pc4_raw;

static void MeterADC_GPIO_Init(void)
{
    GPIO_InitTypeDef gi;

    /* PA1：电压表/欧姆表 */
    gi.GPIO_Mode = GPIO_Mode_In;
    gi.GPIO_Pin  = GPIO_Pin_1;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOA, &gi);

    GPIO_DigitalRemapConfig(
        AFIOA,
        GPIO_Pin_1,
        AFIO_AF_0,
        DISABLE
    );
    GPIO_AnalogRemapConfig(AFIOA, GPIO_Pin_1, ENABLE);

    /* PD2：电流表 */
    gi.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gi);

    GPIO_DigitalRemapConfig(
        AFIOD,
        GPIO_Pin_2,
        AFIO_AF_0,
        DISABLE
    );
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_2, ENABLE);

    /* PC4：电池电压 */
    gi.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOC, &gi);

    GPIO_DigitalRemapConfig(
        AFIOC,
        GPIO_Pin_4,
        AFIO_AF_0,
        DISABLE
    );
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_4, ENABLE);
}

static void MeterADC_DriverInit(void)
{
    ADC_InitTypeDef adc;

    ADC_Cmd(ADC, DISABLE);

    ADC_StructInit(&adc);
    adc.ADC_Prescaler = 48;
    adc.ADC_Mode = ADC_Mode_Single;
    adc.ADC_TriggerSource = ADC_TriggerSource_Software;
    adc.ADC_TimerTriggerSource =
        ADC_TimerTriggerSource_TIM1ADC;
    adc.ADC_Align = ADC_Align_Left;
    adc.ADC_Channel = ADC_CH_VOLT_OHM;
    adc.ADC_ReferencePositive =
        ADC_ReferencePositive_VDD;
    
    // delay_ms(10);
    ADC_Init(ADC, &adc);

    ADC_ScanChannelConfig(
        ADC,
        ADC_CH_VOLT_OHM,
        0
    );
    ADC_ScanChannelConfig(
        ADC,
        ADC_CH_AMP,
        1
    );
    ADC_ScanChannelConfig(
        ADC,
        ADC_CH_BATT,
        2
    );

    ADC_ScanChannelNumberConfig(ADC, 3);
    ADC_ScanCmd(ADC, ENABLE);

    ADC_AverageTimesConfig(ADC, ADC_AverageTimes_16);
    ADC_AverageCmd(ADC, ENABLE);

    ADC_Cmd(ADC, ENABLE);

    while (!ADC_GetFlagStatus(ADC, ADC_FLAG_RDY))
    {
    }
}

void MeterADC_Init(void)
{
    MeterADC_GPIO_Init();
    MeterADC_DriverInit();
}

static void MeterADC_ScanOnce(void)
{
    ADC_StartOfConversion(ADC);

    while (!ADC_GetFlagStatus(ADC, ADC_FLAG_EOS))
    {
    }

    int16_t pa1_data = (int16_t)ADC_GetScanData(ADC, ADC_ScanChannel_0);

    if (pa1_data <= 0)
    {
        s_adc_pa1_raw = 0;
    }
    else
    {
        s_adc_pa1_raw = (uint16_t)(pa1_data >> 3);
    }

    s_adc_pd2_raw =
        (uint16_t)(ADC_GetScanData(
            ADC,
            ADC_ScanChannel_1
        ) >> 3);

    s_adc_pc4_raw =
        (uint16_t)(ADC_GetScanData(
            ADC,
            ADC_ScanChannel_2
        ) >> 3);
}

static uint16_t MeterADC_ReadAverage(uint8_t channel, uint8_t samples)
{
    uint32_t sum = 0;

    if (samples == 0U)
    {
        samples = 1U;
    }

    for (uint8_t i = 0; i < samples; i++)
    {
        MeterADC_ScanOnce();

        switch (channel)
        {
        case 0:
            sum += s_adc_pa1_raw;
            break;

        case 1:
            sum += s_adc_pd2_raw;
            break;

        case 2:
            sum += s_adc_pc4_raw;
            break;

        default:
            break;
        }
    }

    return (uint16_t)(sum / samples);
}

uint16_t MeterADC_ReadPA1(uint8_t samples)
{
    return MeterADC_ReadAverage(0, samples);
}

uint16_t MeterADC_ReadPD2(uint8_t samples)
{
    return MeterADC_ReadAverage(1, samples);
}

uint16_t MeterADC_ReadPC4(uint8_t samples)
{
    return MeterADC_ReadAverage(2, samples);
}

float MeterADC_RawToVoltage(uint16_t raw)
{
    return (float)raw * METER_ADC_VREF / (float)METER_ADC_MAX_RAW;
}