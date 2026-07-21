#include "PT32Y003x.h"
#include <stdbool.h>
#include "uart.h"
#include "multimeter.h"
#include "voltmeter.h"
#include "ammeter.h"
#include "ohmmeter.h"
#include "meter_adc.h"
#include "delay.h"
#include "lcd_ht1621b.h"
#include <PT32Y003x_gpio.h>
#include <PT32Y003x_nvic.h>
#include <PT32Y003x_tim.h>
#include <PT32Y003x_uart.h>
#include <PT32Y003x_adc.h>
#include <PT32Y003x_pwr.h>
#include <PT32Y003x_pwm.h>
#include <PT32Y003x_exti.h>
#include <string.h>

#pragma region 宏定义、全局变量和工具函数配置

// 读到的ADC原始数据 测量端和电池
// static uint16_t g_adc_pa1_raw = 0;  // PA1 电压表输入 / 欧姆表输入
// static uint16_t g_adc_pd2_raw = 0;  // 电流表输入
static uint16_t g_adc_pc4_raw = 0;  // PC4 电池电压输入
// 关机请求
volatile uint8_t poweroff_request = 0;
volatile uint8_t g_require_release_before_poweroff = 0; // 0=未要求, 1=要求先松手
extern uint8_t s_lock_until_release; // 松手锁

// Idle 监控（120s自动休眠功能）
typedef struct {
    float     last_v;
    uint8_t   have_last;
    uint8_t   quiet;          // 1=当前处于“变化率<阈值”的安静区
    uint32_t  quiet_since_ms; // 进入安静区的时间戳
} idle_tracker_t;
volatile idle_tracker_t g_idle = {0};

#define IDLE_WINDOW_MS        (120000U)  // 2min
#define CHANGE_THRESHOLD_ON   (0.10f)     // 进入安静判定阈值（10%）
#define CHANGE_THRESHOLD_OFF  (0.11f)     // 退出安静的回差阈值（11%：轻微迟滞，抗抖）

volatile uint8_t g_short_press_event = 0;

// 长按按键初始化和按键唤醒
// 可调参数
const uint32_t PWR_DEBOUNCE_MS = 90U;
const uint32_t PWR_LONGPRESS_MS = 900U;
const uint32_t POSTWAKE_LONGPRESS_TIMEOUT = 5000U;
// PC5 按键定时器 TIM2 每 10ms 扫描用到的计数
volatile uint16_t s_pwr_stable_ticks = 0;
volatile uint16_t s_pwr_press_ticks  = 0;
volatile uint8_t  s_pwr_last_sample  = 1;   // 1=未按, 0=按下

// 万用表初始模式已设置，开机设置一次
extern bool hadSetMultiMeterMode;
// 万用表工作需要的外设已配置，休眠逻辑相关变量
extern bool hadSetMultimeterInit;

// 系统运行模式：工作态 唤醒态 等待态
typedef enum { RUN_MODE_NORMALWORK = 0, RUN_MODE_DEEPSLEEP = 1, RUN_MODE_WAKEUP = 2} run_mode_t;
volatile run_mode_t g_run_mode = RUN_MODE_NORMALWORK;

// LCD 显示配置
#define LCD_UPDATE_MS         600U
// === LCD 显示缓冲区 ===
struct LCD_BUF_STRUCT
{
    uint32_t last_update_ms;
    /// @brief 显示的数字 必定是正数 0000~9999
    uint16_t num4;
    uint8_t dotpos;
    uint8_t mA_overf_neg_A_V_O_kO;
    uint8_t bat_25_50_75_100_MO;
};
static struct LCD_BUF_STRUCT g_lcd_buf;

// === 电池电量检测 ===
#define BATT_ADC_CHANNEL      ADC_Channel_7   // ★ PC4 对应的 ADC 通道（若不对，请改）
#define BATT_SAMPLE_PERIOD_MS 1000U           // ★ 每秒一次
#define BATT_SAMPLES_N        3
// ★ 电量分档阈值（按你给的门限，单位：V，针对PC4测得的电压）
#define BATT_TH_4             1.55f
#define BATT_TH_3             1.40f
#define BATT_TH_2             1.25f
#define BATT_TH_1             1.15f
// === 电池电量状态机 ===
static struct {
    uint32_t next_ms;
    float    v_filt;   // 低通后的电压
    int      level;    // 0..4
} g_batt;

// ===== 功能函数声明 =====
void first_init(void);
void deep_sleep(void);
void LCD_DelayOneFrame(void);

// 空闲自动睡眠检测
static inline void Idle_OnDisplaySample(float v, uint32_t now_ms)
{
    // 防止“时钟回拨”导致负差值
    if ((int32_t)(now_ms - g_idle.quiet_since_ms) < 0) {
        g_idle.quiet_since_ms = now_ms;
    }

    if (!g_idle.have_last) {
        g_idle.last_v = v;
        g_idle.have_last = 1;
        g_idle.quiet = 1;                // 初始认为进入安静
        g_idle.quiet_since_ms = now_ms;
        return;
    }

    float denom = fabsf(g_idle.last_v);
    if (denom < 0.6f) denom = 0.6f;    // 防 0/极小分母；按你的量纲给个合理下限
    float diff = fabsf(v - g_idle.last_v) / denom;
    g_idle.last_v = v;

    // 迟滞判断：<10% 维持安静；>11% 认为活跃；中间区保持原状态
    if (diff > CHANGE_THRESHOLD_OFF) {
        g_idle.quiet = 0;
        g_idle.quiet_since_ms = now_ms;  // 活跃→重置计时
    } else if (diff < CHANGE_THRESHOLD_ON) {
        if (!g_idle.quiet) {
            g_idle.quiet = 1;            // 刚进入安静→从此刻开始计时
            g_idle.quiet_since_ms = now_ms;
        }
        // 已处于安静：检查是否达到了窗口
        if (g_idle.quiet &&
            (now_ms - g_idle.quiet_since_ms >= IDLE_WINDOW_MS) &&
            g_run_mode == RUN_MODE_NORMALWORK) {
#if ENABLE_LOG
            // debug模式下，不进入睡眠模式
#else
            deep_sleep();
#endif
        }
    }
}

#pragma endregion

#pragma region 自动休眠和按键逻辑
// PC5 长按按键输入配置
static void PowerKey_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_In;
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Pull = GPIO_Pull_Up;   // 上拉，按下为低
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}
static void PowerKey_ResetCounters()
{
    s_pwr_stable_ticks = 0;
    s_pwr_press_ticks  = 0;
    s_pwr_last_sample  = GPIO_ReadDataBit(GPIOC, GPIO_Pin_5) == RESET ? 0 : 1;// 假设未按状态，等消抖
}
static void TIM2_ENABLE(bool flag)
{
    TIM_ClearFlag(TIM2, TIM_FLAG_ARF);
    if (flag)
    {
        TIM_ITConfig(TIM2, TIM_IT_ARI, ENABLE);
        TIM_Cmd(TIM2, ENABLE);
    }
    else
    {
        TIM_ITConfig(TIM2, TIM_IT_ARI, DISABLE);
        TIM_Cmd(TIM2, DISABLE);
    }
}
// 目标：TIM2 产生 ~10ms 周期中断（100Hz）
static void TIM2_Init_10ms(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIMB;

    // ===== 1) 获取 TIM2 的时钟频率 =====
    // 视芯片时钟树而定：很多 M0/M3 定时器挂在 APB（PCLK）或直接 SYSCLK/HCLK。
    // 下面依次尝试，按你 SDK 的可用函数自行取舍：
    uint32_t tim_clk = 0;

    // 若有专门的定时器时钟查询函数，请优先用它（示例）：
    // tim_clk = RCC_GetClockFreq(RCC_TIM2CLK);

    // 否则用 PCLK 或 HCLK 作为 TIM2 时钟来源（两者其一）：
    if (tim_clk == 0) tim_clk = RCC_GetClockFreq(RCC_PCLK);   // 常见：TIM2 挂 APB1
    if (tim_clk == 0) tim_clk = RCC_GetClockFreq(RCC_HCLK);    // 退路：用 HCLK

    if (tim_clk == 0) {
        // 极端防御：若还是拿不到，默认按 48MHz 算，确保不会除以 0
        tim_clk = 48000000UL;
    }

    // ===== 2) 计算 PSC/ARR，使得溢出周期接近 10ms =====
    // 我们要 100Hz：period_counts = tim_clk / 100
    uint32_t target_hz = 100U; // 10ms
    uint32_t period_counts = (tim_clk + target_hz/2) / target_hz; // 四舍五入

    // 约束：ARR、PSC 都是 16bit（0..65535），且实际分频=PSC+1
    // 选择一个尽量小的 PSC，使 ARR 不超过 65535
    uint32_t psc = (period_counts + 65535U) / 65536U;   // 向上取，使 ARR<=65535
    if (psc > 65535U) psc = 65535U;                     // 夹紧
    uint32_t arr = (period_counts / (psc + 1U));
    if (arr == 0) arr = 1;
    if (arr > 65535U) arr = 65535U;

    // 为了让周期更贴近 10ms，再做一次细调（可选）
    uint32_t actual_counts = (psc + 1U) * arr;
    // 如果差距较大，可在此处微调 arr/psc；通常已足够，不必再复杂化

    // ===== 3) 初始化 TIM2 =====
    TIMB.TIM_Prescaler = (uint16_t)psc;
    TIMB.TIM_AutoReload = (uint16_t)(arr - 1U);  // 注意多数库是写 ARR = N-1
    TIMB.TIM_Direction  = TIM_Direction_Up;
    TIM_TimeBaseInit(TIM2, &TIMB);
    TIM2_ENABLE(true);
    // 让TIM2在休眠时也能做长按计时
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
// 配置 PC5 的外部中断，用于休眠唤醒功能
static void Wake_Key_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
    //GPIO端口中断触发类型选择
	EXTI_TriggerTypeConfig(EXTIC,GPIO_Pin_5,EXTI_Trigger_RisingFalling);

	EXTI_ITConfig(EXTIC,GPIO_Pin_5,ENABLE);		//GPIO端口中断使能
	/*使能GPIO的NVIC控制器*/
	NVIC_InitStruct.NVIC_IRQChannel = EXTIC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
}
// 清除 PC5 的外部中断
static void Wake_Key_EXITDisable(void)
{
    EXTI_ITConfig(EXTIC, GPIO_Pin_5, DISABLE);
    EXTI_ClearFlag(EXTIC, GPIO_Pin_5);
    NVIC_DisableIRQ(EXTIC_IRQn);
}
// 关闭GPIO 只保留 PC5 为输入上拉
static inline void deep_sleep_close_gpio(void)
{
    // 关闭屏幕
    GPIO_AnalogRemapConfig(AFIOA, GPIO_Pin_All, DISABLE);
    GPIO_AnalogRemapConfig(AFIOB, GPIO_Pin_All, DISABLE);
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_All, DISABLE);
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_All, DISABLE);
    // SWD接口不可清除复用
    GPIO_DigitalRemapConfig(AFIOA, GPIO_Pin_All,AFIO_AF_None,DISABLE);
    GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_All&(~GPIO_Pin_1),AFIO_AF_None,DISABLE);
    GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_All&(~GPIO_Pin_7),AFIO_AF_None,DISABLE);
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_All&(~GPIO_Pin_1),AFIO_AF_None,DISABLE);
    // 关闭GPIO 只保留 PC5
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;	
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_In;	
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Down;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All&(~GPIO_Pin_5);//WAKE KEY
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
// 进入休眠状态的逻辑，关闭外设
void deep_sleep(void)
{
    if(g_run_mode == RUN_MODE_NORMALWORK)
    {
        // 休眠提示音
        PWM_Cmd(TIM1, ENABLE);
        delay_ms(30);
        PWM_Cmd(TIM1, DISABLE);

        HT1621_Clear();
#if ENABLE_LOG
        LOGF("DEEPSLEEP ms_ticks=%u\r\n", s_ms_ticks);
#endif
        // 等待PC5按键松开
        while (GPIO_ReadDataBit(GPIOC,GPIO_Pin_5)==0)
        {
            delay_ms(10);
        }

        TIM2_ENABLE(false);

        deep_sleep_close_gpio();
#if ENABLE_LOG
        // 关闭 UART
        UART_Cmd(LOG_UART, DISABLE);
#endif
        // 打开外部中断 配置PC5为唤醒源
        Wake_Key_Init();
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // 进睡前关闭系统定时器

        g_run_mode = RUN_MODE_DEEPSLEEP;
        poweroff_request = 0;
        // 重置参考电压2V的输出，下次初始化时重新启动2V参考电压输出，集成正向版万用表，移除参考电压输出
        // ADC_BGCRResetBGNC(ADC);
        // 进入深度睡眠
        PWR_EnterDeepSleepMode(PWR_DeepSleepEntry_WFI);
        g_run_mode = RUN_MODE_WAKEUP;
        // —— 从 EXTI 唤醒返回 —— 关闭唤醒用 EXTI，避免运行态乱中断
        Wake_Key_EXITDisable();

        first_init();
        // 记录时间戳
        uint32_t t0 = s_ms_ticks;
#if ENABLE_LOG
        UART_Driver();
        UART_Cmd(LOG_UART, ENABLE);

        LOGF("WAKEUP ticks=%u\r\n",s_ms_ticks);
#endif
        // 在唤醒后保持按住 2 秒回到工作态 若未长按 5s后重新睡眠 由TIM2中断服务程序修改系统运行状态
        while(g_run_mode == RUN_MODE_WAKEUP)
        {
            // 到时未确认 -> 回睡 5000ms即5秒
            if ((s_ms_ticks - t0) >= POSTWAKE_LONGPRESS_TIMEOUT) {
                TIM2_ENABLE(false);
                g_run_mode = RUN_MODE_NORMALWORK;
                LOGS("sleep again\r\n");
                deep_sleep();
                return; // 不会走到这里
            }
        }

        // idle_last_ms = s_ms_ticks;//重置变化率<10%的120s计数
#if ENABLE_LOG
        LOGF("NORMALWORK ms_ticks=%u\r\n",s_ms_ticks);
#endif

        // 返回正常工作，清除标志位，此时需要再次调用万用表外设配置函数
        hadSetMultimeterInit = false;
    }
}
#pragma endregion

#pragma region 初始化和屏幕显示
// LCD 初始化
void LCDInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // CS
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OutPP;
	GPIO_InitStruct.GPIO_Pull = GPIO_Pull_Up;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
    // WR
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OutPP;
	GPIO_InitStruct.GPIO_Pull = GPIO_Pull_Up;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
    // DATA
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OutPP;
	GPIO_InitStruct.GPIO_Pull = GPIO_Pull_Up;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    HT1621_Init();
}
void LCD_DelayOneFrame(void)
{
    g_lcd_buf.last_update_ms = s_ms_ticks + LCD_UPDATE_MS;
}
static void LCD_DISPLAY_UPDATE(void)
{
    // 1宏定义了 200ms 更新屏幕
    uint32_t now = s_ms_ticks;
    if ((int32_t)(now - g_lcd_buf.last_update_ms) < 0) return;
    g_lcd_buf.last_update_ms = now + LCD_UPDATE_MS;

    // 清除负号/溢出
    g_lcd_buf.mA_overf_neg_A_V_O_kO &= ~(ICON_NEG|ICON_OVERF);
    uint32_t scaled = 0;
    uint8_t dotpos  = 0;
    // 更新对应的表的图标、负号、溢出
    switch (Multimeter_GetMode())
    {
    case METER_MODE_VOLT:
        {
            float voltage = Voltmeter_GetVoltage();
            // 空闲变化率<10%的休眠判断，用PA1换算后的电压判断
            Idle_OnDisplaySample(Voltmeter_GetADCVoltage(), s_ms_ticks);
            if (voltage > 0.0f)
            {
                scaled = (uint32_t)(voltage * 100.0f + 0.5f);
                if (voltage >= VOLT_MAX_V)
                {
                    g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
            
                    scaled = 1200;
                }
            }
            else
            {
                scaled = 0;
            }

            dotpos = 2;
            break;
        }
    case METER_MODE_AMP:
        // 先清掉量纲位（A、mA），避免上一模式残留
        g_lcd_buf.mA_overf_neg_A_V_O_kO &= ~(ICON_AMP_A<<4 | ICON_AMP_MA);
        // 使用“统一”显示：始终以 A 为单位，保留 3 位小数 → num4=|I|*1000, dotpos=1
        {
            float i = Ammeter_GetCurrent();
            Idle_OnDisplaySample(Ammeter_GetADCVoltage(), s_ms_ticks);
            // 取绝对值
            bool neg = (i < 0.0f);
            if (neg) {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_NEG;
                i=-i;
            }
            // 显示 1.xxx A
            if (i >= 1.0f)
            {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_AMP_A<<4;
                dotpos = 1;
                // 之前由于零点漂移，实际上到不了3A，只能的到2.8A 2.94f 现在可以到3A
                if (i >= AMP_FULLSCALE_A){
                    // 显示溢出
                    g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                    i = AMP_FULLSCALE_A;
                }
            }
            else
            {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_AMP_MA;
            }
            scaled = (uint32_t)(i * 1000.0f + 0.5f);
        }
        break;
    case METER_MODE_OHM:
        // 清掉电压/电流/Ω系图标，保留电池外框
        g_lcd_buf.mA_overf_neg_A_V_O_kO &= ~(ICON_OHM<<4 | ICON_OHM_KO<<4);
        // 第二字节中，仅清除 MΩ 位
        g_lcd_buf.bat_25_50_75_100_MO &= ~(ICON_OHM_MO<<4);
        {
            // 尚未转化为四位数字的原始电阻值
            float rx = Ohmmeter_GetResistance();
            Idle_OnDisplaySample(Ohmmeter_GetADCVoltage(), s_ms_ticks);
            
            if (rx < 51.0f) {
                if (rx < 1.0f) {
                    rx = 0.0f;
                }
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                if (!(TIM1->CR1 & 1) && Ohmmeter_GetState()== OHMMETER_MEASURE)
                {
                    PWM_Cmd(TIM1, ENABLE);
                }
            }
            else
            {
                if (TIM1->CR1 & 1)
                {
                    PWM_Cmd(TIM1, DISABLE);
                }
            }

            if (rx <= 999.49f) {
                // 0 ~ 999 Ω，整数显示
                scaled = (uint32_t)(rx + 0.5f);
                dotpos = 0;
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OHM<<4;      // Ω

            } else if (rx < 100000.0f) {
                // 1.00k ~ 99.99k
                float v_k = rx / 1000.0f;
                if (v_k > 99.99f) v_k = 99.99f;
                scaled = (uint32_t)(v_k * 100.0f + 0.5f); // xx.xx
                dotpos = 2;
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OHM_KO<<4;   // kΩ

            } else if (rx < 1000000.0f) {
                // 100.0k ~ 999.9k
                float v_k = rx / 1000.0f;
                if (v_k > 999.9f) v_k = 999.9f;
                scaled = (uint32_t)(v_k * 10.0f + 0.5f); // xxx.x
                dotpos = 3;
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OHM_KO<<4;   // kΩ

            } else {
                // 1.00M ~ 51.00M
                float v_M = rx / 1000000.0f;
                if (v_M >= 51.00f) {
                    v_M = 51.00f;
                    g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                    dotpos = 4;
                }else
                {
                    dotpos = 2;
                }
                scaled = (uint32_t)(v_M * 100.0f + 0.5f); // xx.xx
                g_lcd_buf.bat_25_50_75_100_MO |= ICON_OHM_MO<<4;     // MΩ（在第二字节）
            }
        }
        break;

    default:
        break;
    }
    // 清除电池电量（保持外框）
    g_lcd_buf.bat_25_50_75_100_MO &= ~((ICON_BAT_25 << 4) | (ICON_BAT_50 << 4) | ICON_BAT_75 | ICON_BAT_100);
    // 显示电池电量
    switch (g_batt.level)
    {
    case 4:
        g_lcd_buf.bat_25_50_75_100_MO |= ICON_BAT_100;
    case 3:
        g_lcd_buf.bat_25_50_75_100_MO |= ICON_BAT_75;
    case 2:
        g_lcd_buf.bat_25_50_75_100_MO |= ICON_BAT_50<<4;
    case 1:
        g_lcd_buf.bat_25_50_75_100_MO |= ICON_BAT_25<<4;
        break;
    default:
        break;
    }
    g_lcd_buf.dotpos = dotpos;
    g_lcd_buf.num4 = (uint16_t)scaled;
    // 更新四位数字和小数点位置
    LCD_Show_digits(g_lcd_buf.num4, g_lcd_buf.dotpos);
    LCD_ShowIcon(g_lcd_buf.mA_overf_neg_A_V_O_kO, g_lcd_buf.bat_25_50_75_100_MO);
}

#pragma endregion

#pragma region 电池电量检测
static int Battery_LevelFromV(float v)
{
    if (v >= BATT_TH_4) return 4;
    if (v >= BATT_TH_3) return 3;
    if (v >= BATT_TH_2) return 2;
    if (v >= BATT_TH_1) return 1;
    return 0;
}
// 电池电量监测任务初始化
void BatteryTask_Init(void)
{
    g_batt.next_ms = s_ms_ticks;
    g_batt.level   = -1;     // 未定级

    // 常久显示电池外框
    g_lcd_buf.bat_25_50_75_100_MO |= ICON_BAT_BROAD;
}

void BatteryTask_Update(void)
{
    if ((int32_t)(s_ms_ticks - g_batt.next_ms) < 0) return;
    g_batt.next_ms = s_ms_ticks + BATT_SAMPLE_PERIOD_MS;
    // 读取ADC原始值
    g_adc_pc4_raw = MeterADC_ReadPC4(BATT_SAMPLES_N);
    // 转成电池电压 1.5V 外部电阻做了分压
    float v = 2 * MeterADC_RawToVoltage(g_adc_pc4_raw);

    int lvl = Battery_LevelFromV(v);
#if ENABLE_LOG
    // LOGF("BATT: %dV [%d/4]\r\n", (int)(v*1000.0f+0.5f), lvl);
#endif

    g_batt.level = lvl;
}
#pragma endregion

#pragma region 蜂鸣器初始化
// 蜂鸣器初始化
void BuzzerInit(void)
{
    GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_6, AFIO_AF_2,ENABLE);//CH1

    PWM_TimeBaseInitTypeDef PWM_TimeBaseInitType;
	PWM_OCInitTypeDef OutInit;

	/* 时钟选择 */
	PWM_TimeBaseInitType.PWM_ClockSource = PWM_ClockSource_SYSCLK;
	/* 中央计数模式 -- 不开启 */
	PWM_TimeBaseInitType.PWM_CenterAlignedMode = PWM_CenterAlignedMode_Disable;
	/* 计数器计数模式，设置为向上计数 */
	PWM_TimeBaseInitType.PWM_Direction = PWM_Direction_Up;
	/* 周期匹配寄存器,累计MR0+1个频率后产生一个更新或者中断 根据驱动计数器1M 计算加载值369时 恰好2.7kHz符合蜂鸣器的最佳频率*/
	PWM_TimeBaseInitType.PWM_AutoReloadValue = 369;// 369
	/* 驱动CNT计数器的时钟 = Fcksys/(psc+1) 48M分频后变成1M*/ 
	PWM_TimeBaseInitType.PWM_Prescaler = 47;

    /* 初始化TIM1*/
	PWM_TimeBaseInit(TIM1,&PWM_TimeBaseInitType);

	/* 配置为PWM输出通道为1通道*/
	OutInit.PWM_Channel = PWM_Channel_1;
    /* 配置为PWM输出模式 */	
	OutInit.PWM_OCMode = TIM_OCMode_PWM1;
    /* 配置输出 */	
	OutInit.PWM_OCOutput = PWM_OCOutput_Enable;
    /* 设置PWM空闲时候的输出电平状态 */
    OutInit.PWM_OCIdleState = PWM_OCIdleState_Low;
    OutInit.PWM_OCValue = 185;
    /* 配置PWM比较输出极性*/	
    OutInit.PWM_OCPolarity = PWM_OCPolarity_High;

    // 如果你不用互补通道，显式关闭也可以（可写可不写，StructInit 已给默认值）：
    OutInit.PWM_OCNOutput    = PWM_OCNOutput_Disable;
    OutInit.PWM_OCNPolarity  = PWM_OCNPolarity_Low;
    OutInit.PWM_OCNIdleState = PWM_OCNIdleState_Low;

    PWM_OCInit(TIM1, &OutInit);
}
#pragma endregion

#pragma region 主循环逻辑
// 定时器时钟初始化
void first_init(void)
{
    SysTick_Init_1kHz();// 系统时钟定时器 us ms 计时已测试 准确
    
    PowerKey_GPIO_Init(); // 长按开关机的按键输入配置
    TIM2_Init_10ms();// 长按时间定时器TIM2

    PowerKey_ResetCounters();// 长按时间计数清零
}
void TaskInit(void)
{
    BuzzerInit();
    LCDInit();
    // 唤醒/初始化后：复位空闲检测器 和 LCD显示缓冲区
    memset((void*)&g_idle, 0, sizeof(g_idle));
    memset((void*)&g_lcd_buf, 0, sizeof(g_lcd_buf));
    BatteryTask_Init();
    Multimeter_Init();
    Multimeter_SetMode(Multimeter_GetMode());
    if (Multimeter_GetMode()==METER_MODE_VOLT)
    {
        g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_VOLT<<4;
    }
    hadSetMultimeterInit = true;
    // 短按提示音
    PWM_Cmd(TIM1, ENABLE);
    delay_ms(25);
    PWM_Cmd(TIM1, DISABLE);
}
int main(void)
{
    meter_mode_t meter_mode;

    first_init();

#if ENABLE_LOG
    // uart0_tx 串口日志 PD5 、uart1_tx 串口日志 PB1
    UART_Driver();
    LOGS("UART Init");
    // 开机立刻工作
    TaskInit();
#else
    // 开机立刻休眠
    deep_sleep();
#endif
    
    while (1)
    {
        if (g_run_mode == RUN_MODE_NORMALWORK)
        {
            // 从休眠时唤醒也会走到这个任务初始化里面
            if(!hadSetMultimeterInit)
            {
                TaskInit();
            }
            if (g_short_press_event)
            {
                g_short_press_event = 0;
                Multimeter_NextMode();
                // 清LCD 图标缓冲，避免残留
                g_lcd_buf.mA_overf_neg_A_V_O_kO = 0;
                g_lcd_buf.bat_25_50_75_100_MO &= ICON_BAT_BROAD;
                if (Multimeter_GetMode()==METER_MODE_VOLT)
                {
                    g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_VOLT<<4;
                }
                // 切换万用表类型的短按提示音
                PWM_Cmd(TIM1, ENABLE);
                delay_ms(25);
                PWM_Cmd(TIM1, DISABLE);
            }
            // 更新万用表的数据
            Multimeter_Update();
            // 更新电池电量
            BatteryTask_Update();
            // 更新LCD显示
            LCD_DISPLAY_UPDATE();
    
            if (poweroff_request && !g_require_release_before_poweroff)
            {
                deep_sleep();
            }
        }
        else if (g_run_mode == RUN_MODE_DEEPSLEEP)
        {
            if (!g_require_release_before_poweroff) {
                poweroff_request = 1;                     // 只在已松手过后才允许关机
                s_lock_until_release = 1;
            }
        }
        else if (g_run_mode == RUN_MODE_WAKEUP)
        {
            g_run_mode = RUN_MODE_NORMALWORK;
            s_lock_until_release = 1;                     // 仍需等松手
            g_require_release_before_poweroff = 1;        // ★ 进入工作态后必须先松手一次
        }
    }
}
#pragma endregion
/*
1 LSB ≈ 3.0V / 4095 ≈ 0.732mV

电流表
R26 = 0.1Ω ÷ 3W
运放增益 ≈ 10.1
ADC 参考 = VDD，实际约 3.0V
PD2 读取电流表 ADC
有效测量起点：约 0.1A 以上
主要可用范围：0.1A ~ 2.8A
2.8A 左右接近 ADC 满量程，按 3A 满量程处理
工程标称满量程：3A

电压表
待测电压 → 510k → PA1 → 150k → Q5 → GND
U5.2 运放跟随
PA1 读取电压表 ADC
Vin = Vpa1 × (510k + 150k) ÷ 150k
Vin = Vpa1 × 4.4
主要可用范围：约 1V ~ 12V
建议标称范围：0V ~ 12V 正向电压
实际较可靠范围：3V ~ 12V
1V 附近误差偏大，后续做零点低压补偿
反向电压：不测量，不显示负数

欧姆表
短路需要交给 Ω 档判断。
开路/高阻需要交给 MΩ 档判断。
测量范围最大3MΩ。
*/

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(u8* file, u32 line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
	/* Infinite loop */
	while (1)
	{
	}
}
#endif


