// meter_mode
//     SwitchMeterMode
//     MultimeterInit
//     三种表之间的硬件模式切换
#include "multimeter.h"

#include "meter_adc.h"
#include "voltmeter.h"
#include "ammeter.h"
#include "ohmmeter.h"
// #include "lcd_ht1621b.h"

#include "PT32Y003x.h"
#include <PT32Y003x_gpio.h>
#include <stdbool.h>

static meter_mode_t s_mode = METER_MODE_VOLT;

// 万用表初始模式已设置，开机设置一次
bool hadSetMultiMeterMode = false;
// 万用表工作需要的外设已配置，休眠逻辑相关变量
bool hadSetMultimeterInit = false;

void MultimeterChangeGPIOConfig(void)
{
    GPIO_InitTypeDef gi;
    // 初始化 PD3 PD4 引脚，用于切换电压表、电流表、欧姆表
    gi.GPIO_Mode = GPIO_Mode_OutPP;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    gi.GPIO_Pin  = GPIO_Pin_3;
    GPIO_Init(GPIOD, &gi);
    gi.GPIO_Pin  = GPIO_Pin_4;
    GPIO_Init(GPIOD, &gi);

    // // 关闭模拟功能
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_3, DISABLE);
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_4, DISABLE);

    // 关闭数字外设复用
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_3, AFIO_AF_1, DISABLE);
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_4, AFIO_AF_1, DISABLE);
}

void Multimeter_Init(void)
{
    // ADC引脚配置，扫描通道初始化
    MeterADC_Init();
    // PD3 PD4 用于切换三种表
    MultimeterChangeGPIOConfig();
    // 关闭欧姆档换档的三个IO引脚，避免干扰其他功能引脚
    Ohmmeter_GPIO_Init();
    Ohmmeter_AllOff();
    // 如果没有设置模式，就设置成电压表模式
    if (!hadSetMultiMeterMode)
    {
        s_mode = METER_MODE_VOLT;
        hadSetMultiMeterMode = true;
    }
}

void Multimeter_SetMode(meter_mode_t mode)
{
    if (s_mode == mode && hadSetMultimeterInit) return;

    s_mode = mode;

    switch (s_mode)
    {
    case METER_MODE_VOLT:
        Ohmmeter_AllOff();

        GPIO_SetBits(GPIOD, GPIO_Pin_3);
        GPIO_ResetBits(GPIOD, GPIO_Pin_4);

        Voltmeter_Init();
        break;

    case METER_MODE_AMP:
        Ohmmeter_AllOff();

        GPIO_SetBits(GPIOD, GPIO_Pin_3);
        GPIO_ResetBits(GPIOD, GPIO_Pin_4);

        Ammeter_Init();
        break;

    case METER_MODE_OHM:
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
        GPIO_SetBits(GPIOD, GPIO_Pin_4);

        Ohmmeter_Init();
        break;

    default:
        break;
    }
}

void Multimeter_NextMode(void)
{
    switch (s_mode)
    {
    case METER_MODE_VOLT:
        Multimeter_SetMode(METER_MODE_AMP);
        break;

    case METER_MODE_AMP:
        Multimeter_SetMode(METER_MODE_OHM);
        break;

    case METER_MODE_OHM:
    default:
        Multimeter_SetMode(METER_MODE_VOLT);
        break;
    }
}

void Multimeter_Update(void)
{
    switch (s_mode)
    {
    case METER_MODE_VOLT:
        Voltmeter_Update();
        break;

    case METER_MODE_AMP:
        Ammeter_Update();
        break;

    case METER_MODE_OHM:
        Ohmmeter_Update();
        break;

    default:
        break;
    }
}

meter_mode_t Multimeter_GetMode(void)
{
    return s_mode;
}