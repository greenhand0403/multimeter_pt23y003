#include "PT32Y003x.h"
#include <stdbool.h>
#include "ledDisplay.h"
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
#define ENABLE_LOG 0
#if ENABLE_LOG
char log_buffer[64];  // 用于打印日志 足够存储格式化字符串
  #define LOG_UART UART0
  #define LOGF(...) do{ sprintf(log_buffer, __VA_ARGS__); UART1_SendString(log_buffer); }while(0)
  #define LOGS(s)   do{ UART1_SendString(s); }while(0)
#else
  #define LOGF(...) do{}while(0)
  #define LOGS(s)   do{}while(0)
#endif

// 读到的ADC原始数据 本来是全局的，给判断变化率10%使用的
static uint16_t g_adc_pa1_raw = 0;  // 序号0（PA1）
static uint16_t g_adc_pc4_raw = 0;  // 序号1（PC4）
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
// 长按按键初始化和按键唤醒
// 可调参数
const uint32_t PWR_DEBOUNCE_MS = 90U;
const uint32_t PWR_LONGPRESS_MS = 900U;
const uint32_t POSTWAKE_LONGPRESS_TIMEOUT = 5000U;
// PC5 按键定时器 TIM2 每 10ms 扫描用到的计数
volatile uint16_t s_pwr_stable_ticks = 0;
volatile uint16_t s_pwr_press_ticks  = 0;
volatile uint8_t  s_pwr_last_sample  = 1;   // 1=未按, 0=按下

// 万用表初始模式已设置
static bool hadSetMultiMeterMode = false;
// 万用表工作需要的外设已配置
static bool hadSetMultimeterInit = false;
// 运行模式：工作态 唤醒态 等待态
typedef enum { RUN_MODE_NORMALWORK = 0, RUN_MODE_DEEPSLEEP = 1, RUN_MODE_WAKEUP = 2} run_mode_t;
volatile uint8_t g_run_mode = RUN_MODE_NORMALWORK;   // 默认处于休眠模式

// 万用表类型： 电压表 电流表 欧姆表
typedef enum { METER_MODE_VOLT = 0, METER_MODE_AMP = 1,METER_MODE_OHM = 2 } meter_mode_t;
static meter_mode_t meter_mode = -1;

// 开机零点 电流表
float V_REF = 0.994f;
// 开机零点 电流表
// float V_REF = 0.994f;
static inline float V_DV(float v_raw) { return v_raw - V_REF; }

#define ADC_TO_V(x)   ((x) * 2.0f / 4095.0f) // ADC 原始值转电压
#define AVG_N            5 // 平均采样次数
// 电流表
#define I_ZERO_OFFSET_A     0.f   // 正向零点漂移
#define I_FULLSCALE_A       3.0f    // 满量程（|I| 的上限）

// 正向（0~3A）：读数偏高 +5~+50mA 开机 减去5mA
#define ERR_POS_AT0_A       0.004f  // +5mA @ ~0A
#define ERR_POS_ATFS_A      0.024f // +50mA @ +3A

// 反向（-3~0A）：读数偏低 ?5~?50mA（等价于数值更“负”）
// 用正数表示“误差幅度”，方向由符号统一处理
#define ERR_NEG_AT0_A       0.004f  // 10mA @ ~0A
#define ERR_NEG_ATFS_A      0.024f  // 50mA @ -3A

// 小电流死区（抗抖），可按噪声调整
#define I_DEADBAND_A        0.0025f  // 2.5mA
// 硬件与标定数值
#define RSHUNT           0.1f      // 采样电阻
#define I_IDLE_A         0.006f    // <6mA 视为无负载
// A 档（默认）
#define GAIN_A           3.9f
// mA 档（带你的斜率修正系数）存在误差，大约只放大了30倍而非34倍
#define GAIN_mA          33.99f
#define MA_SLOPE_FIX     10.f   // 你前面标定得出的斜率系数
// mA 档上限（=28mA）
#define I_MA_MAX         0.28f
#define ZERO_BAND_V       0.0040f                    // 零点死区：|ΔV|<4mV 视为0V
// 欧姆表
// #define VIN_ZERO_TH         0.33f    // <此电压视为短路(10Ω)
#define VIN_ZERO_TH         0.96f    // <此电压视为短路(50Ω)

// 采样电阻（含你之前微调可继续放在这里统一管理）
#define RS_OHM_RAW          51.0f     // 51Ω
#define RS_KOHM_RAW         5100.0f     // 5.1kΩ
#define RS_MOHM_RAW         510000.0f     // 510kΩ

// 分档校准（斜率/零点），后续实测再填；默认1与0表示未校准
// Ω 档（≤510Ω）
#define GAIN_OHM    1.0255f
#define OFFS_OHM    0.0f

// kΩ 档（0.51k ~ 51kΩ）
#define GAIN_KOHM   1.0494f
#define OFFS_KOHM   (-135.691f)

// MΩ 档分段（≥75kΩ）
#define MOHM_SPLIT_OHMS   220000.0f   // 220k 为分界
#define GAIN_MOHM_LOW     1.0197f   // 75k~220k
#define OFFS_MOHM_LOW     (-253.17f)
#define GAIN_MOHM_HIGH    1.0591f   // ≥470k
#define OFFS_MOHM_HIGH    (-30288.99f)
// 电阻表档位
typedef enum { RANGE_OHM = 0, RANGE_KOHM, RANGE_MOHM } ohm_range_t;
// LCD 显示
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
// === 电压表采样间隔时间 ===
#ifndef VOLT_SAMPLE_PERIOD_MS
#define VOLT_SAMPLE_PERIOD_MS   300U    // 电流 电压 欧姆表 更新周期：300ms
#endif
// 电压表分压公式
#ifndef K_VOLT_SLOPE
// ((1000.0f+81.0f)/81.0f/2.0f)
// #define K_VOLT_SLOPE            6.66f
// ((1000.0f+81.0f)/81.0f*2.0f)
#define K_VOLT_SLOPE            26.39f
#endif
#define VOLT_ZERO_OFFSET     +0.07f   // 正向零点漂移
// 是否做上/下限钳位（例如 0~12V）
#ifndef VOLT_MAX_V
#define VOLT_MAX_V              12.0f
#endif
#ifndef VOLT_MIN_V
#define VOLT_MIN_V              (-12.0f)
#endif
// === 电压表状态机 ===
typedef struct {
    uint32_t next_ms;   // 下次允许采样的时间戳(ms)
    float    last_v;    // 上一帧电压，供抖动/保留显示使用（可选）
} volt_ctx_t;

static volt_ctx_t g_volt;
// ===== 电流表状态机 =====
typedef enum { AMP_S_MEASURE_mA, AMP_S_MEASURE_A } amp_state_t;

static struct {
    amp_state_t st;
    bool mAflag;     // false=A 档, true=mA 档
    float vin, iamp; // 最近一次的测量数据
} g_amp;
// ===== 欧姆表状态机 =====
typedef enum { OHM_S_SELECT_RANGE = 0, OHM_S_MEASURE } ohm_state_t;

static struct {
    ohm_state_t st;
    ohm_range_t range;
    float vin;
    float rx_display;
} g_ohm;
// ===== 功能函数声明 =====
void first_init(void);
void deep_sleep(void);
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
            
#else
            deep_sleep();
#endif
        }
    }
}

#pragma endregion
#pragma region 串口驱动
#if ENABLE_LOG
/*******************************************************************************
*Function:	UART_GPIO_Config
*Description:	配置UART引脚
*Input:		无
*Return:		无
*Others:
			该函数负责使能UART模块相关引脚
*******************************************************************************/
void UART_GPIO_Config(void)
{

	/* 配置UART管脚的复用功能 */
    if (LOG_UART==UART1)
    {
        GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PB1 TX1
        GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PD1 RX1
    }
    else
    {
        GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0,ENABLE);	//PD5 TX0
        GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0,ENABLE);	//PD6 RX0
    }

}

/*******************************************************************************
*Function:	UART_Mode_Config
*Description:	配置UART
*Input:		无
*Return:		无
*Others:
			该函数负责初始化UART模块的工作及其工作方式
*******************************************************************************/
void UART_Mode_Config(void)
{

	UART_InitTypeDef  UART_InitStruct;

	/*初始化UART0*/
	UART_InitStruct.UART_BaudRate = 9600;
	UART_InitStruct.UART_WordLengthAndParity=UART_WordLengthAndParity_8D;
	UART_InitStruct.UART_StopBitLength=UART_StopBitLength_1;
	UART_InitStruct.UART_ParityMode=UART_ParityMode_Odd;
	UART_InitStruct.UART_Receiver=UART_Receiver_Enable;
	UART_InitStruct.UART_LoopbackMode=UART_LoopbackMode_Disable;

    /*开启收发功能*/
	UART_Cmd(LOG_UART, ENABLE);
    UART_Init(LOG_UART, &UART_InitStruct);

}
/*******************************************************************************
*Function:	UART_Driver
*Description:	UART模块驱动函数
*Input:		无
*Return:		无
*Others:
*******************************************************************************/
void UART_Driver(void)
{
	UART_GPIO_Config();
	UART_Mode_Config();
}

// 串口发送字符串函数
void UART1_SendString(const char* str)
{
    while (*str)
    {
        UART_SendData(LOG_UART, *str++);
        while (UART_GetFlagStatus(LOG_UART, UART_FLAG_TXE) == RESET);
    }
}
#endif
#pragma endregion
#pragma region 参考电压输出配置
/*******************************************************************************
*Function:	ADC_Mode_Config
*Description:	配置ADC
*Input:		无
*Return:		无
*Others:
该函数负责初始化ADC模块的工作及其工作方式
*******************************************************************************/
void ADC_Driver(void)
{
    ADC_Cmd(ADC, DISABLE);
    
	ADC_InitTypeDef  ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_Prescaler = 48;						 	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Single;						//单次转换模式
	ADC_InitStruct.ADC_TriggerSource = ADC_TriggerSource_Software;
	ADC_InitStruct.ADC_TimerTriggerSource=ADC_TimerTriggerSource_TIM1ADC;//定时源触发选择TIM0事件
	ADC_InitStruct.ADC_Align = ADC_Align_Left;					//左对齐
	ADC_InitStruct.ADC_Channel = ADC_Channel_1;//PA1
	ADC_InitStruct.ADC_BGVoltage=ADC_BGVoltage_BG1v0;//BGS电压1.0v
	ADC_InitStruct.ADC_ReferencePositive = ADC_ReferencePositive_BG2v0;
	ADC_BGCRSetBGNC(ADC);// SET ADC_BGNC BIT
    
    delay_ms(10);

	ADC_Init(ADC, &ADC_InitStruct);

    // ★ 扫描序列：序号0=PA1(ADC1)【测量端】，序号1=PC4(ADC7)【电池】
    ADC_ScanChannelConfig(ADC, ADC_Channel_1, 0);
    ADC_ScanChannelConfig(ADC, BATT_ADC_CHANNEL, 1);
    ADC_ScanChannelNumberConfig(ADC, 2);
    ADC_ScanCmd(ADC, ENABLE);

    // （可选）硬件平均
    ADC_AverageTimesConfig(ADC, ADC_AverageTimes_16);
    ADC_AverageCmd(ADC, ENABLE);

    ADC_Cmd(ADC, ENABLE);
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_RDY));
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

        LOGF("DEEPSLEEP ms_ticks=%u\r\n", s_ms_ticks);
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
        // 重置参考电压2V的输出，下次初始化时重新启动2V参考电压输出
        ADC_BGCRResetBGNC(ADC);
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
#endif
        LOGF("WAKEUP ticks=%u\r\n",s_ms_ticks);

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

        LOGF("NORMALWORK ms_ticks=%u\r\n",s_ms_ticks);

        // 返回正常工作，清除标志位，此时需要再次调用万用表外设配置函数
        hadSetMultimeterInit = false;
    }
}
#pragma endregion
#pragma region 万用表类型判断配置
void MultiMeterIOOutputConfig(bool enable)
{
	if (!enable)
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_In;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
		GPIO_InitStruct.GPIO_Pull=GPIO_Pull_Down;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
		GPIO_DigitalRemapConfig(AFIOA, GPIO_Pin_2, AFIO_AF_None,DISABLE);

		GPIO_InitTypeDef GPIO_InitStruct2;
		GPIO_InitStruct2.GPIO_Mode=GPIO_Mode_In;
		GPIO_InitStruct2.GPIO_Pin=GPIO_Pin_3;
		GPIO_InitStruct2.GPIO_Pull=GPIO_Pull_Down;
		GPIO_Init(GPIOA,&GPIO_InitStruct2);
		GPIO_DigitalRemapConfig(AFIOA, GPIO_Pin_3, AFIO_AF_None,DISABLE);
	}else
    {
        GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OutPP;
		GPIO_InitStruct.GPIO_Pull = GPIO_Pull_NoPull;	//无偏置
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OutPP;
		GPIO_InitStruct.GPIO_Pull = GPIO_Pull_NoPull;	//无偏置
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}
#pragma endregion
#pragma region 万用表读取电压和输出配置
// 扫描 ADC 并更新 g_adc_pa1_raw 和 g_adc_pc4_raw
static void ADC_ScanOnce(void)
{
    ADC_StartOfConversion(ADC);
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_EOS));           // 等“扫描完成”

    // 注意：取“扫描序号”而不是“物理通道号”
    uint16_t d0 = (uint16_t)ADC_GetScanData(ADC, ADC_ScanChannel_0);
    uint16_t d1 = (uint16_t)ADC_GetScanData(ADC, ADC_ScanChannel_1);

    g_adc_pa1_raw = (d0 >> 3);   // 左对齐 → 还原到 12bit 范围
    g_adc_pc4_raw = (d1 >> 3);
}

static float read_vin(int n)
{
    uint32_t acc = 0;
    for (int i = 0; i < n; ++i) {
        ADC_ScanOnce();
        acc += g_adc_pa1_raw;
    }
    uint16_t raw = (uint16_t)(acc / (uint32_t)n);
    return ADC_TO_V(raw); // = raw * 2.0 / 4095（继续使用你的宏）
}

// MΩ 档对 Rs 的微调（按你此前经验：低/中/高阻做轻微补偿，可选）
static inline float tune_Rs_Mohm(float vin)
{
    float Rs = RS_MOHM_RAW;
    if (vin > 1.34f)      Rs = RS_MOHM_RAW * 0.992f;   // 高阻
    else if (vin < 0.76f) Rs = RS_MOHM_RAW * 1.0153f; // 低阻
    return Rs;
}

static inline float compute_rx(float vin, float Rs)
{
    // 经典电阻表：Rx = Rs * Vin / (Vref - Vin)，Vref=2.0V
    return Rs * vin / (2.0f - vin);
}

static void set_range_pins(ohm_range_t r)
{
    switch (r)
    {
        case RANGE_OHM:   // Ω 档：PA2=0, PA3=1
            GPIO_ResetBits(GPIOA, GPIO_Pin_2);
            GPIO_SetBits(GPIOA,   GPIO_Pin_3);
            break;
        case RANGE_KOHM:  // kΩ 档：PA2=1, PA3=0
            GPIO_SetBits(GPIOA,   GPIO_Pin_2);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            break;
        case RANGE_MOHM:  // MΩ 档：PA2=1, PA3=1（或按你硬件）
            GPIO_SetBits(GPIOA,   GPIO_Pin_2);
            GPIO_SetBits(GPIOA,   GPIO_Pin_3);
            break;
    }
}

static inline float _interp_err(float iabs, float e0, float efs, float ifs) {
    // 线性插值：e(i) = e0 + (efs - e0) * (i/ifs), 0<=i<=ifs
    if (iabs < 0.0f) iabs = 0.0f;
    if (iabs > ifs) iabs = ifs;
    return e0 + (efs - e0) * (iabs / ifs);
}
#pragma endregion
#pragma region 初始化和屏幕显示

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
    switch (meter_mode)
    {
    case METER_MODE_VOLT:
        if ( g_volt.last_v > 0.0f) {
            scaled = (uint16_t)(g_volt.last_v * 100.0f);
            if (g_volt.last_v >= VOLT_MAX_V)
            {
                // 显示溢出
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                scaled = 1200;
            }
        } else {
            scaled = (uint16_t)(g_volt.last_v * -100.0f);
            g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_NEG;
            if (g_volt.last_v <= VOLT_MIN_V)
            {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                scaled = 1200;
            }
        }
        dotpos = 2;
        // 空闲变化率<10%的休眠判断
        Idle_OnDisplaySample(scaled, s_ms_ticks);
        break;
    case METER_MODE_AMP:
        // 先清掉量纲位（A、mA），避免上一模式残留
        g_lcd_buf.mA_overf_neg_A_V_O_kO &= ~(ICON_AMP_A<<4 | ICON_AMP_MA);
        // 使用“统一”显示：始终以 A 为单位，保留 3 位小数 → num4=|I|*1000, dotpos=1
        {
            float i = g_amp.iamp;
            Idle_OnDisplaySample(g_amp.vin, s_ms_ticks);
            // 取绝对值
            bool neg = (i < 0.0f);
            if (neg) {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_NEG;
                i=-i;
            }
            // 显示 1.xxx A
            if (i > 0.999f)
            {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_AMP_A<<4;
                dotpos = 1;
                // 由于零点漂移，实际上到不了3A，只能的到2.8A
                if (i >= 2.95f){
                    // 显示溢出
                    g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                    i = I_FULLSCALE_A;
                }
            }
            else
            {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_AMP_MA;
            }
            scaled = (uint16_t)(i * 1000.0f + 0.5f);
        }
        break;
    case METER_MODE_OHM:
        // 清掉电压/电流/Ω系图标，保留电池外框
        g_lcd_buf.mA_overf_neg_A_V_O_kO &= ~(ICON_OHM<<4 | ICON_OHM_KO<<4);
        // 第二字节中，仅清除 MΩ 位
        g_lcd_buf.bat_25_50_75_100_MO &= ~(ICON_OHM_MO<<4);
        {
            // 尚未转化为四位数字的原始电阻值
            float rx = g_ohm.rx_display;
            Idle_OnDisplaySample(g_ohm.vin, s_ms_ticks);
            
            if (rx < 51.0f) {
                if (rx < 1.0f) {
                    rx = 0.0f;
                }
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_OVERF;
                if (!(TIM1->CR1 & 1))// g_ohm.st == OHM_S_MEASURE &&
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
// ★ PC4 接 ADC：打开模拟复用 + 输入无上下拉
static void Battery_GPIO_Init(void)
{
    GPIO_InitTypeDef gi;
    gi.GPIO_Mode = GPIO_Mode_In;
    gi.GPIO_Pin  = GPIO_Pin_4;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOC, &gi);
    GPIO_DigitalRemapConfig(AFIOC,GPIO_Pin_4,AFIO_AF_0,DISABLE);
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_4, ENABLE);  // PC4→ADC
}

static int Battery_LevelFromV(float v)
{
    if (v >= BATT_TH_4) return 4;
    if (v >= BATT_TH_3) return 3;
    if (v >= BATT_TH_2) return 2;
    if (v >= BATT_TH_1) return 1;
    return 0;
}

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

    uint32_t adc_sum = 0;
    // 取 PC4 的电压；如需平均可循环 ADC_ScanOnce() 多次求平均
    for (uint8_t i = 0; i < BATT_SAMPLES_N; i++)
    {
        ADC_ScanOnce();
        adc_sum += g_adc_pc4_raw;
    }
    g_adc_pc4_raw = adc_sum / BATT_SAMPLES_N;
    // 转成电压 电池1.5V 做了分压
    float v = 2 * ADC_TO_V(g_adc_pc4_raw);

    int lvl = Battery_LevelFromV(v);
    
    // LOGF("BATT: %dV [%d/4]\r\n", (int)(v*1000.0f+0.5f), lvl);

    g_batt.level = lvl;
}
#pragma endregion

#pragma region 蜂鸣器初始化
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
#pragma region 电压表业务逻辑

// ——（可选）你的“2分钟 <10% 变化自动休眠”的检测 ——
// 若已集成 IdleDetector_*，可在 Update 里调用 IdleDetector_Update(v_meas);

// 小工具：把 dv -> 物理电压（V）
static inline float Volt_From_DV(float dv) {
    return dv * K_VOLT_SLOPE;
}

void VoltTask_Init(void)
{
    g_volt.next_ms = s_ms_ticks;  // 立即可以更新
    g_volt.last_v  = 0.0f;

    // 固定显示符号 V
    g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_VOLT<<4;
}

/* ========== 单步更新：无阻塞、无死循环 ========== */
void VoltTask_Update(void)
{
    // 1) 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((int32_t)(now - g_volt.next_ms) < VOLT_SAMPLE_PERIOD_MS) return;
    g_volt.next_ms = now;

    float v_raw = read_vin(AVG_N);
    // LOGF("ticks=%u v=%d idle=%d\r\n", s_ms_ticks,(int)(v_raw*1000.0f+0.5f),idle_last_ms);

    // 对应换算公式是 ( vout - 0.9983 ) * 10000.0 / 379.2
    float dv = V_DV(v_raw);
    // 减缓微小的波动
    float v_tmp = Volt_From_DV(dv);    // 真实输入（V，带正负号）
    // 正向零点漂移
    v_tmp += VOLT_ZERO_OFFSET;
    
    float vabs = fabsf(v_tmp);
    // 门限电压
    if (vabs <= 0.04f) 
        v_tmp = 0.0f;
    else {
    // ③ 正/反向线性误差模型扣除
    float e = _interp_err(vabs, -0.04, +0.04, VOLT_MAX_V);
    if (v_tmp >= 0.04f) {
        // 正向误差：读数“偏高” → 需要减去一个值
        vabs -= e;
    } else {
        // 反向误差：读数“偏低”（更负） → 需要加回一个值
        vabs += e;
    }
    v_tmp = v_tmp >= 0.0f ? vabs : -vabs;
}
    // float delta = fabs(v_tmp-g_volt.last_v);
    // if (delta <= 0.05f)
    // {
    //     return;
    // }
    // else if (delta >= 0.1f)
    // {
    //     g_volt.last_v = v_tmp;
    // }else
    // {
    //     // 正电压零点漂移
    //     if (g_volt.last_v > 0.0f)
    //     {
    //         g_volt.last_v -= 0.1f;
    //     }
    //     g_volt.last_v += delta * 0.5f;
    // }
    
    // // 舍弃微小电压
    // if (fabs(g_volt.last_v) <= 0.05f)
    // {
    //     g_volt.last_v = 0.0f;
    // }
    g_volt.last_v = v_tmp;
}
#pragma endregion
#pragma region 电流表业务逻辑
void AmpTask_Init(void)
{
    // PA2 作为量程控制：0=A档，1=mA档
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OutPP;
    GPIO_InitStruct.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_SetBits(GPIOA, GPIO_Pin_2); // 默认 mA 档
    g_amp.mAflag = true;
    g_amp.st     = AMP_S_MEASURE_mA;
    g_amp.iamp = 0.0f;

    g_volt.next_ms = s_ms_ticks;  // 立即可以更新
    LOGS("Amp init: mA-range (PA2=1)\r\n");
}
static inline float current_compensate(float i_meas)
{
    float i = i_meas;
    // ① 正向零点抵消
    // if (i>0.f)
    // {
    //     i-= I_ZERO_OFFSET_A;
    // }
    // 抵消零点误差后，如果是微小的电流，视为 0mA
    if (fabsf(i) <= I_DEADBAND_A) return 0.0f;

    // ③ 正/反向分开线性误差模型并扣除
    float iabs = fabsf(i);
    if (i >= 0.0f) {
        // 正向误差：读数“偏高” → 需要减去一个正的幅值
        float e = _interp_err(iabs, ERR_POS_AT0_A, ERR_POS_ATFS_A, I_FULLSCALE_A);
        i -= e;
        if (i<0.f)
        {
            i=0.f;
        }
    } else {
        // 反向误差：读数“偏低”（更负） → 需要加回一个幅值
        float e = _interp_err(iabs, ERR_NEG_AT0_A, ERR_NEG_ATFS_A, I_FULLSCALE_A);
        i += e;
        if (i>0.f)
        {
            i=0.f;
        }
    }

    return i;
}
/*
void AmpTask_Update(void)
{
    // 1) 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((int32_t)(now - g_volt.next_ms) < VOLT_SAMPLE_PERIOD_MS) return;
    g_volt.next_ms = now;
    // 无滤波
    g_amp.vin = read_vin(AVG_N);

    float dv = V_DV(g_amp.vin);
    if (fabsf(dv) < ZERO_BAND_V) {                // ★ 零点死区：|ΔV|<4mV 视为 0
        if (g_amp.mAflag)
        {
            g_amp.iamp = 0.0f;
        } else if (fabsf(dv) < (I_MA_MAX * 0.1f * GAIN_A)) {
            g_amp.mAflag = true;
            GPIO_SetBits(GPIOA, GPIO_Pin_2);
            g_amp.st = AMP_S_MEASURE_mA;
        }
        return; 
    }
    // 电流表改成差分输入，因此需要乘 2.0f 得到最终的电流
    switch (g_amp.st)
    {
    case AMP_S_MEASURE_mA:
        g_amp.iamp = dv * MA_SLOPE_FIX / GAIN_mA;// mA放大了34倍
        // 正向电压有偏置，反映到测量电流上，导致正向电流由0.004A的零点漂移，反向则没有
        // if (g_amp.iamp>0.f)
        // {
        //     if (g_amp.iamp<0.005f)
        //     {
        //         g_amp.iamp *= 0.4f;
        //     }else
        //     {
        //         g_amp.iamp -= 0.004f;
        //     }
        // }
        LOGF("mA=%d v=%d\r\n", (int)(g_amp.iamp * 1000.0f+0.5f), (int)(g_amp.vin * 1000.0f + 0.5f));
        // 退出条件
        if (g_amp.iamp >= I_MA_MAX||g_amp.iamp <= -I_MA_MAX) {
            // 超 mA 档上限 -> 重新判档 mA档只能测到280mA
            LOGS("mA->A\r\n");
            GPIO_ResetBits(GPIOA, GPIO_Pin_2);
            g_amp.mAflag = false;
            g_amp.st     = AMP_S_MEASURE_A;
        }
        
        break;

    case AMP_S_MEASURE_A:
        g_amp.iamp = dv * 10.0f / GAIN_A; // A放大了4倍
        // 误差补偿 10mA
        // if (g_amp.iamp<1.f)
        // {
        //     g_amp.iamp += 0.01f;
        // }
        LOGF("A=%d v=%d\r\n", (int)(g_amp.iamp * 1000.0f+0.5f), (int)(g_amp.vin * 1000.0f + 0.5f));
        // 退出条件：A档不测小电流，A档要测到280mA以上的
        // if (g_amp.iamp <= I_MA_MAX && g_amp.iamp >= -I_MA_MAX) {
        //     // 超 A 档下限 -> 重新判档 A档不测小电流，A档要测到294mA以上的
        //     LOGS("A->mA\r\n");
        //     GPIO_SetBits(GPIOA, GPIO_Pin_2);
        //     g_amp.mAflag = true;
        //     g_amp.st = AMP_S_MEASURE_mA;
        // }
        // 超 A 档上限
        if (g_amp.iamp >= I_FULLSCALE_A) {
            LOGS("OVER: >=2.5A\r\n");
            g_amp.iamp = I_FULLSCALE_A;
        }else if (g_amp.iamp <= -I_FULLSCALE_A) {
            LOGS("OVER: <=-2.5A\r\n");
            g_amp.iamp = -I_FULLSCALE_A;
        }
        
        break;
    }
}
*/
void AmpTask_Update2(void)
{
    // 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((int32_t)(now - g_volt.next_ms) < VOLT_SAMPLE_PERIOD_MS) return;
    g_volt.next_ms = now;

    g_amp.vin  = read_vin(AVG_N);
    
    g_amp.iamp = V_DV(g_amp.vin) / 3.33f * 10.0f; // A放大了4倍
    g_amp.iamp = current_compensate(g_amp.iamp);  // ★ 误差补偿

    // 超 A 档上限
    if (g_amp.iamp >= I_FULLSCALE_A) {
        LOGS("OVER: >=2.5A\r\n");
        g_amp.iamp = I_FULLSCALE_A;
    }else if (g_amp.iamp <= -I_FULLSCALE_A) {
        LOGS("OVER: <=-2.5A\r\n");
        g_amp.iamp = -I_FULLSCALE_A;
    }
    LOGF("mA=%d v=%d\r\n", (int)(g_amp.iamp * 1000.0f+0.5f), (int)(g_amp.vin * 1000.0f + 0.5f));
}
#pragma endregion
#pragma region 欧姆表业务逻辑
void OhmTask_Init(void)
{
    // 51MΩ
    g_ohm.rx_display = 51000000.0f;
    g_volt.next_ms = s_ms_ticks;

    MultiMeterIOOutputConfig(true); // 需要控制PA2/PA3时转为输出
    g_ohm.range = RANGE_KOHM; // 开机默认处于kΩ档，然后根据电压切到 Ω或MΩ档
    g_ohm.st = OHM_S_SELECT_RANGE;

    set_range_pins(g_ohm.range);
    // LOGS("Ohm init\r\n");

    // 重置LCD刷新的计时器，防止初始化时刷新屏幕
    g_lcd_buf.last_update_ms = g_volt.next_ms + VOLT_SAMPLE_PERIOD_MS;
}

// 每次调用仅推进一步；无阻塞、无 while(1)
void OhmTask_Update(void)
{   
    // 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((uint32_t)(now - g_volt.next_ms) < VOLT_SAMPLE_PERIOD_MS) return;
    g_volt.next_ms = now;

    if (g_ohm.st == OHM_S_SELECT_RANGE) {
        if (g_ohm.range != RANGE_KOHM)
        {
            g_ohm.range = RANGE_KOHM;
            set_range_pins(g_ohm.range);
        }
        g_ohm.vin = read_vin(AVG_N);
        // kΩ档 测量 510Ω 0.176 530Ω 0.181以下的电阻 切Ω档
        if (g_ohm.vin<0.179f)
        {
            g_ohm.range = RANGE_OHM;
            set_range_pins(g_ohm.range);
        }
        // kΩ档 测量 51kΩ以上的电阻 切MΩ档
        else if (g_ohm.vin>1.817f)
        {
            g_ohm.range = RANGE_MOHM;
            set_range_pins(g_ohm.range);
        }
        // 测量完成 切到测量状态
        g_ohm.st = OHM_S_MEASURE;
    } else {
        g_ohm.vin = read_vin(AVG_N);

        // Ω档忽略小电压波动
        if (g_ohm.range == RANGE_OHM && g_ohm.vin <= 0.1f)
        {
            g_ohm.rx_display = 0;
            return;
        }

        // 计算 Rx（原始）
        float Rs = (g_ohm.range == RANGE_OHM)  ? RS_OHM_RAW :
                (g_ohm.range == RANGE_KOHM) ? RS_KOHM_RAW : RS_MOHM_RAW;
        float rx_raw = compute_rx(g_ohm.vin, Rs);
        float rx = rx_raw;

        // ===== 分档校准 =====
        if (g_ohm.range == RANGE_OHM) {
            rx = rx * GAIN_OHM + OFFS_OHM;
        } else if (g_ohm.range == RANGE_KOHM) {
            rx = rx * GAIN_KOHM + OFFS_KOHM;
        } else {
            // MΩ 档按区间分段
            if (rx_raw <= MOHM_SPLIT_OHMS) {
                rx = rx_raw * GAIN_MOHM_LOW  + OFFS_MOHM_LOW;
            } else {
                rx = rx_raw * GAIN_MOHM_HIGH + OFFS_MOHM_HIGH;
            }
        }

        g_ohm.rx_display = rx;
        
        // 超出测量范围 时回去选档
        switch (g_ohm.range)
        {
        case RANGE_OHM:
            if (rx > 510.0f) {
                g_ohm.st = OHM_S_SELECT_RANGE;
                // 重置LCD刷新的计时器，防止换挡时刷新屏和报警
                g_lcd_buf.last_update_ms = now + VOLT_SAMPLE_PERIOD_MS;
            }
            break;
        case RANGE_KOHM:
            if (rx < 510.0f || rx > 51000.0f) {
                g_ohm.st = OHM_S_SELECT_RANGE;
            }
            break;
        case RANGE_MOHM:
            if (rx < 51000.0f) {
                g_ohm.st = OHM_S_SELECT_RANGE;
            }
            break;
        default:
            break;
        }

    }
}

#pragma endregion
#pragma region 万用表初始化
void check_meter_mode(void)
{
    if (GPIO_ReadDataBit(GPIOA,GPIO_Pin_2)==RESET)
    {
        if (GPIO_ReadDataBit(GPIOA,GPIO_Pin_3)==RESET)
        {
            LOGS("Volt Mode\r\n");
            meter_mode = METER_MODE_VOLT;
        }
        else
        {
            LOGS("Amp Mode\r\n");
            meter_mode = METER_MODE_AMP;
        }
    }
    else
    {
        LOGS("Ohm Mode\r\n");
        meter_mode = METER_MODE_OHM;
    }
}

void MultimeterInit()
{
    Battery_GPIO_Init();
    ADC_Driver();// PA1 和 PC4 作为 ADC 输入

    if (!hadSetMultiMeterMode)
    {
        LOGS("Meter IO Init");
        // 配置 PA2 PA3 输入模式 根据情况选择电流表、电压表或欧姆表
        MultiMeterIOOutputConfig(false);
        // init state: PA2 PA3 , LOW LOW mean VoltTest, LOW HIGH mean AmpTest, HIGH HIGH mean OhmTest
        check_meter_mode();
        hadSetMultiMeterMode = true;
    }

	BuzzerInit();

    LCDInit();
    // 唤醒/重初始化后：复位空闲检测器 和 LCD显示缓冲区
    memset((void*)&g_idle, 0, sizeof(g_idle));
    memset((void*)&g_lcd_buf, 0, sizeof(g_lcd_buf));

    BatteryTask_Init();

    switch (meter_mode)
    {
    case METER_MODE_VOLT:
        VoltTask_Init();
        break;
    case METER_MODE_AMP:
        AmpTask_Init();
        break;
    case METER_MODE_OHM:
        OhmTask_Init();
        break;
    default:
        break;
    }

    // 初始化仪表成功提示音
    PWM_Cmd(TIM1, ENABLE);
    delay_ms(30);
    PWM_Cmd(TIM1, DISABLE);
    
    hadSetMultimeterInit = true;
}
#pragma endregion
#pragma region 主循环逻辑
void first_init(void)
{
    SysTick_Init_1kHz();// 系统时钟定时器 us ms 计时已测试 准确
    
    PowerKey_GPIO_Init(); // 长按开关机的按键输入配置
    TIM2_Init_10ms();// 长按时间定时器TIM2
    PowerKey_ResetCounters();// 长按时间计数清零
}
int main (void)
{
    first_init();
#if ENABLE_LOG
    // uart0_tx 串口日志 PD5 uart1_tx 串口日志 PB1
    UART_Driver();
    LOGS("UART Init");
#endif
#if 0 // 读取开机时的 2V 输出端 电压
    s32 ADC_average=0;
    ADC_InitTypeDef  ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_Prescaler = 64;						 	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Single;						//单次转换模式
	ADC_InitStruct.ADC_TriggerSource = ADC_TriggerSource_Software;
	ADC_InitStruct.ADC_TimerTriggerSource=ADC_TimerTriggerSource_TIM1ADC;						//定时源触发选择TIM0事件
	ADC_InitStruct.ADC_Align = ADC_Align_Left;					//左对齐
	ADC_InitStruct.ADC_Channel=ADC_Channel_12;						
	ADC_InitStruct.ADC_ReferencePositive= ADC_ReferencePositive_BG2v0;	//选择VDDA作为正端参考电平
	ADC_InitStruct.ADC_BGVoltage=ADC_BGVoltage_BG1v0;//BGS电压1.0v
	ADC_Init(ADC, &ADC_InitStruct);
    ADC_Cmd(ADC, ENABLE);		//启动ADC外设功能
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_RDY));	//等待ADC启动完成
    for(uint8_t i=0;i<100;i++)
    {
		ADC_StartOfConversion(ADC);	//启动转换
		while(!ADC_GetFlagStatus(ADC, ADC_FLAG_EOC));		//等待ADC转换完成
		ADC_average += (s16)ADC_GetConversionValue(ADC);	//获取结果
	}
    ADC_average/=100;
	ADC_average >>= 2;//左对齐处理
	V_REF=((float)ADC_average)*2.0/8192;
    LOGF("V_REF:%d\r\n", (int)(V_REF*1000.0f+0.5f));

    // MultimeterInit();
    while (1)
    {
        // g_amp.vin = read_vin(AVG_N);
        // LOGF("g_amp.vin:%d\r\n", (int)(g_amp.vin*1000.0f+0.5f));

        LOGF("g_amp.vin:%d\r\n", (int)(V_REF*10000.0f));

        LCD_Show_digits((int)(V_REF*1000.0f+0.5f), 1);
        delay_ms(1000);
    }
#endif
#if 0
 
    // 测试电阻表三个档位的换挡阈值
    MultimeterInit();
    // g_ohm.range = RANGE_OHM;
    // set_range_pins(g_ohm.range);

    // 进入 mA 档
    // GPIO_SetBits(GPIOA, GPIO_Pin_2);
    // g_amp.mAflag = true;
    // g_amp.st     = AMP_S_MEASURE_mA;
    // 留在 A 档
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    g_amp.mAflag = false;
    g_amp.st     = AMP_S_MEASURE_A;
    while (1)
    {
        delay_ms(300);
        // continue;
        // 读取电压
        // g_ohm.vin = read_vin(AVG_N);

        // 计算 Rx
        // float Rs = (g_ohm.range == RANGE_OHM)  ? RS_OHM_RAW :
        //            (g_ohm.range == RANGE_KOHM) ? RS_KOHM_RAW : RS_MOHM_RAW;
        // float rx = compute_rx(g_ohm.vin, Rs);
        
        // 分档校准
        // if (g_ohm.range == RANGE_OHM)   rx = rx * GAIN_OHM  + OFFS_OHM;
        // if (g_ohm.range == RANGE_KOHM)  rx = rx * GAIN_KOHM + OFFS_KOHM;
        // if (g_ohm.range == RANGE_MOHM)  rx = rx * GAIN_MOHM + OFFS_MOHM;

        // g_ohm.rx_display = rx;

        g_amp.vin  = read_vin(AVG_N);
        if (g_amp.st == AMP_S_MEASURE_mA)
        {
            g_amp.iamp = V_DV(g_amp.vin) / GAIN_mA * MA_SLOPE_FIX;
        }else if (g_amp.st == AMP_S_MEASURE_A)
        {
            g_amp.iamp = (g_amp.vin - 0.99f) / 3.33f * 10.0f; // A放大了4倍
            g_amp.iamp = current_compensate(g_amp.iamp);  // ★ 误差补偿
        }
        
        BatteryTask_Update();     // ★ 每秒打印一次电池电量
        // g_lcd_buf.num4 = (uint16_t)(g_ohm.vin*1000.0f+0.5f);
        // 更新四位数字和小数点位置
        // LCD_Show_digits(g_lcd_buf.num4, g_lcd_buf.dotpos);
        LCD_DISPLAY_UPDATE();
        LOGF("g_amp.vin:%d g_amp.iamp:%d \r\n", (int)(g_amp.vin*1000.0f+0.5f), (int)(g_amp.iamp*1000.0f+0.5f));
    }
#endif
    deep_sleep();//方便测试关闭睡眠
    for (;;)
    {
        if (g_run_mode == RUN_MODE_NORMALWORK)
        {
            if(!hadSetMultimeterInit)
            {
                MultimeterInit();
            }
            switch (meter_mode)
            {
            case METER_MODE_VOLT:
                VoltTask_Update();
                break;
            case METER_MODE_AMP:
                AmpTask_Update2();
                break;
            case METER_MODE_OHM:
                OhmTask_Update();
                break;
            default:
                break;
            }
            BatteryTask_Update();     // ★ 每秒打印一次电池电量
            LCD_DISPLAY_UPDATE();
            if (poweroff_request && !g_require_release_before_poweroff)//要求长按松手后才关机
            {
                LOGS("wait to poweroff");
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


