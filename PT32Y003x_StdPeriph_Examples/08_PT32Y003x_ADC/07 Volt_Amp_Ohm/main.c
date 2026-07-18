#include "PT32Y003x.h"
#include <stdbool.h>
// #include "ledDisplay.h"
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

#define ENABLE_LOG 1
#if ENABLE_LOG
char log_buffer[64];  // 用于打印日志 足够存储格式化字符串
  #define LOG_UART UART0
  #define LOGF(...) do{ sprintf(log_buffer, __VA_ARGS__); UART1_SendString(log_buffer); }while(0)
  #define LOGS(s)   do{ UART1_SendString(s); }while(0)
#else
  #define LOGF(...) do{}while(0)
  #define LOGS(s)   do{}while(0)
#endif

// 读到的ADC原始数据 测量端和电池
static uint16_t g_adc_pa1_raw = 0;  // PA1 电压表输入 / 欧姆表输入
static uint16_t g_adc_pd2_raw = 0;  // 电流表输入
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

// 万用表初始模式已设置
static bool hadSetMultiMeterMode = false;
// 万用表工作需要的外设已配置
static bool hadSetMultimeterInit = false;
// 运行模式：工作态 唤醒态 等待态
typedef enum { RUN_MODE_NORMALWORK = 0, RUN_MODE_DEEPSLEEP = 1, RUN_MODE_WAKEUP = 2} run_mode_t;
volatile run_mode_t g_run_mode = RUN_MODE_NORMALWORK;

// 万用表类型： 电压表 电流表 欧姆表
typedef enum { METER_MODE_VOLT = 0, METER_MODE_AMP = 1,METER_MODE_OHM = 2 } meter_mode_t;
// 开机默认模式
static meter_mode_t meter_mode = METER_MODE_VOLT;

// ===== 集成三表单向测量版引脚定义 ===== 模式切换按键仍然是 PC5 短按切换三种表
#define ADC_CH_VOLT_OHM     ADC_Channel_1        // PA1
#define ADC_CH_AMP          ADC_Channel_6        // PD2
#define ADC_CH_BATT         ADC_Channel_7        // PC4

// ADC 通道平均采样扫描次数 和 电压转换
#define ADC_TO_V(x)   ((x) * 3.0f / 4095.0f) // ADC 原始值转电压
#define AVG_N            5 // 平均采样次数

// 电流表配置
#define I_ZERO_OFFSET_A     0.f   // 正向零点漂移
#define I_FULLSCALE_A       3.0f    // 满量程（|I| 的上限）
// 正向（0~3A）：读数偏高 +5~+50mA 开机 减去5mA
#define ERR_POS_AT0_A       -0.008f  // +5mA @ ~0A
#define ERR_POS_ATFS_A      0.028f // +50mA @ +3A
// 反向（-3~0A）：读数偏低 ?5~?50mA（等价于数值更“负”）
// 用正数表示“误差幅度”，方向由符号统一处理
#define ERR_NEG_AT0_A       -0.008f  // 10mA @ ~0A
#define ERR_NEG_ATFS_A      0.028f  // 50mA @ -3A
// 小电流死区（抗抖），可按噪声调整
// #define I_DEADBAND_A        0.0025f  // 2.5mA
#define I_DEADBAND_A        0.008f  // 8mA
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
#define ZERO_BAND_V       0.0040f   // 零点死区：|ΔV|<4mV 视为0V
// 误差校正参数
#define AMP_ZERO_DEADBAND_A   0.010f
#define AMP_CAL_GAIN       0.9765f
#define AMP_CAL_OFFSET_A  (-0.00535f)
// 开路模拟输出饱和阈值
#define AMP_OPEN_RAW_TH 6000U
// 正常电流上升到约2.9A后，PD2 ADC进入顶部饱和区
// 饱和区统一按3A超量程显示
#define AMP_FULLSCALE_RAW_TH   4050U

// 欧姆表配置
#define OHM_SUPPLY_V       3.0f  // 欧姆表上端电压3V
// #define VIN_ZERO_TH         0.33f    // <此电压视为短路(10Ω)
#define VIN_ZERO_TH         0.96f    // <此电压视为短路(50Ω)
// 采样电阻（含你之前微调可继续放在这里统一管理）
#define RS_OHM_RAW          51.0f     // 51Ω
#define RS_KOHM_RAW         5100.0f     // 5.1kΩ
#define RS_MOHM_RAW         510000.0f     // 510kΩ
#define R_GS_SHUNT_RAW 5000000.0f  // 5MΩ 开路时的电阻
#define OHM_OPEN_VALUE     51000000.0f  // 超出量程，电阻太大，统一表示为51MΩ
// ===== 欧姆表实测参数 =====
// Ω档：短路零点约 0.20~0.29V，先按 0.30V 以下认为短路
#define OHM_SHORT_TH_V          0.30f
// kΩ档实测等效模型
// 实测：短路约0.20V，开路约1.946V，5.1k点反推 Rth≈3.5k
// #define KOHM_VZERO_V            0.20f
// #define KOHM_VOPEN_V            1.946f
// #define KOHM_RTH_OHM            3500.0f
// 自动换挡阈值：先保守一点
#define KOHM_TO_OHM_TH_V        0.45f   // kΩ档低于此值，切Ω档
#define KOHM_TO_MOHM_TH_V       1.80f   // kΩ档高于此值，认为高阻/开路
// 电阻表档位
typedef enum { RANGE_OHM = 0, RANGE_KOHM, RANGE_MOHM } ohm_range_t;
// 千欧档自动选档阈值：基于5.1kΩ参考电阻重新实测
// 使用kΩ档进行总选档
#define KOHM_TO_OHM_RAW        430U
#define KOHM_TO_MOHM_RAW      3850U

// Ω档测量超过上限，返回重新选档
#define OHM_TO_SELECT_RAW      3700U

// MΩ档测量低于下限，返回重新选档
#define MOHM_TO_SELECT_RAW      600U
// MΩ档换挡引脚
#define MOHM_SEL_PIN_PORT           GPIOD
#define MOHM_SEL_PIN_NUM            GPIO_Pin_6

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

// === 电压表采样间隔时间 ===
#ifndef VOLT_SAMPLE_PERIOD_MS
#define VOLT_SAMPLE_PERIOD_MS   300U    // 电流 电压 欧姆表 更新周期：300ms
#endif
// 电压表分压公式
#ifndef K_VOLT_SLOPE
// ((510k+150k)/150k)
#define K_VOLT_SLOPE            4.4f
#endif
#define VOLT_ZERO_OFFSET     +0.0f   // 正向零点漂移
// 是否做上/下限钳位（例如 0~12V）
#ifndef VOLT_MAX_V
#define VOLT_MAX_V              12.0f
#endif
#ifndef VOLT_MIN_V
#define VOLT_MIN_V              (-12.0f)
#endif
// 电压表实测线性标定
#define VOLT_CAL_GAIN          1.01f
#define VOLT_CAL_OFFSET_V     (-0.05f)

// 正向测量版：低于此值认为未接电压
#define VOLT_ZERO_DEADBAND_V   0.10f
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
    uint16_t raw;
    float vin;
    float rx_display;
} g_ohm;

// ===== 功能函数声明 =====
void first_init(void);
void deep_sleep(void);
// 切换三种表的时候，GPIO 引脚对应的进行设置
static void SwitchMeterMode(meter_mode_t new_mode);

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
// 插值计算误差的函数，方便做误差补偿？
static inline float _interp_err(float iabs, float e0, float efs, float ifs) {
    // 线性插值：e(i) = e0 + (efs - e0) * (i/ifs), 0<=i<=ifs
    if (iabs < 0.0f) iabs = 0.0f;
    if (iabs > ifs) iabs = ifs;
    return e0 + (efs - e0) * (iabs / ifs);
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
        // GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PB1 TX1
        // GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PD1 RX1
    }
    else
    {
        GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0,ENABLE);	//PD5 TX0
        // GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0,ENABLE);	//PD6 RX0 会影响PA1！详情就见文档
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
// 串口驱动
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
*Description:	配置ADC扫描通道
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
	ADC_InitStruct.ADC_Channel = ADC_Channel_1;
	ADC_InitStruct.ADC_BGVoltage = ADC_BGVoltage_BG1v2;//BGS电压1.2v
	ADC_InitStruct.ADC_ReferencePositive = ADC_ReferencePositive_VDD;
	// ADC_BGCRSetBGNC(ADC);// SET ADC_BGNC BIT
    
    delay_ms(10);

	ADC_Init(ADC, &ADC_InitStruct);

    // ★ 扫描序列：序号0=PA1(ADC1)【电压表、欧姆表输入端】，序号1PD2电流表输入端，序号2=PC4(ADC7)【电池】
    ADC_ScanChannelConfig(ADC, ADC_CH_VOLT_OHM, 0); // PA1
    ADC_ScanChannelConfig(ADC, ADC_CH_AMP,  1); // PD2
    ADC_ScanChannelConfig(ADC, ADC_CH_BATT, 2); // PC4
    ADC_ScanChannelNumberConfig(ADC, 3);
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

#pragma region 欧姆表控制引脚初始化
// 欧姆表控制引脚初始化 PA2, PA3，PB1, 自动换档
static void OhmCtrl_GPIO_Init(void)
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

static void Ohm_AllOff(void)
{
    // 高电平，关断所有 P-MOS
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_SetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);
}

#pragma endregion

#pragma region 万用表读取电压和输出配置
// 扫描 ADC 并更新 g_adc_pa1_raw, g_adc_pd2_raw, g_adc_pc4_raw
static void ADC_ScanOnce(void)
{
    ADC_StartOfConversion(ADC);
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_EOS));

    uint16_t d0 = (uint16_t)ADC_GetScanData(ADC, ADC_ScanChannel_0);
    uint16_t d1 = (uint16_t)ADC_GetScanData(ADC, ADC_ScanChannel_1);
    uint16_t d2 = (uint16_t)ADC_GetScanData(ADC, ADC_ScanChannel_2);

    g_adc_pa1_raw = (d0 >> 3);// 左对齐 → 还原到 12bit 范围 0~4096
    g_adc_pd2_raw = (d1 >> 3);
    g_adc_pc4_raw = (d2 >> 3);
}

static float read_vin(meter_mode_t src, int n)
{
    uint32_t acc = 0;
    for (int i = 0; i < n; ++i) {
        ADC_ScanOnce();
        switch (src) {
            case METER_MODE_AMP:
                acc += g_adc_pd2_raw; break;
            // 欧姆表和电压表都用PA1通道
            case METER_MODE_VOLT:
            case METER_MODE_OHM:
                acc += g_adc_pa1_raw; break;
            default:
                break;
        }
    }
    uint16_t raw = (uint16_t)(acc / (uint32_t)n);
    return ADC_TO_V(raw);
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
            scaled = 0;
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
            if (i >= 1.0f)
            {
                g_lcd_buf.mA_overf_neg_A_V_O_kO |= ICON_AMP_A<<4;
                dotpos = 1;
                // 之前由于零点漂移，实际上到不了3A，只能的到2.8A 2.94f 现在可以到3A
                if (i >= I_FULLSCALE_A){
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
                if (!(TIM1->CR1 & 1) && g_ohm.st == OHM_S_MEASURE)
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

// 配置三个输入IO口和电池电压输入
static void MeterADC_GPIO_Init(void)
{
    GPIO_InitTypeDef gi;

    // PA1 -> 电压表输入 / 欧姆表输入
    gi.GPIO_Mode = GPIO_Mode_In;
    gi.GPIO_Pin  = GPIO_Pin_1;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOA, &gi);
    GPIO_DigitalRemapConfig(AFIOA, GPIO_Pin_1, AFIO_AF_0, DISABLE);
    GPIO_AnalogRemapConfig(AFIOA, GPIO_Pin_1, ENABLE);

    // PD2 -> 电流表输入
    gi.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gi);
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_2, AFIO_AF_0, DISABLE);
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_2, ENABLE);

    // PC4 -> 电池
    gi.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOC, &gi);
    GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_4, AFIO_AF_0, DISABLE);
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_4, ENABLE);

    // 初始化 PD3 PD4 引脚，用于切换电压表、电流表、欧姆表
    gi.GPIO_Mode = GPIO_Mode_OutPP;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    gi.GPIO_Pin  = GPIO_Pin_3;
    GPIO_Init(GPIOD, &gi);
    gi.GPIO_Pin  = GPIO_Pin_4;
    GPIO_Init(GPIOD, &gi);

    // 关闭模拟功能
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_3, DISABLE);
    GPIO_AnalogRemapConfig(AFIOD, GPIO_Pin_4, DISABLE);

    // 关闭数字外设复用
    GPIO_DigitalRemapConfig(
        AFIOD,
        GPIO_Pin_3,
        AFIO_AF_None,
        DISABLE
    );

    GPIO_DigitalRemapConfig(
        AFIOD,
        GPIO_Pin_4,
        AFIO_AF_None,
        DISABLE
    );

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

    LOGF("PA1_raw:%d PD2_raw:%d meter_mode:%d\r\n",(int)g_adc_pa1_raw, (int)g_adc_pd2_raw, meter_mode);

    switch (meter_mode)
    {
    case METER_MODE_VOLT:
        LOGF("g_volt.last_v:%d\r\n", (int)(g_volt.last_v*1000.0f)); 
        break;
    case METER_MODE_AMP:
        LOGF("g_amp.iamp:%d\r\n", (int)(g_amp.iamp*1000.0f)); 
        break;
    case METER_MODE_OHM:
        LOGF("range:%d raw:%d vin_mV:%d RX:%d\r\n",
            (int)g_ohm.range,
            (int)g_ohm.raw,
            (int)(g_ohm.vin * 1000.0f + 0.5f),
            (int)(g_ohm.rx_display + 0.5f));
        break;
    default:
        break;
    }
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

#pragma region 电压表业务逻辑

// ——（可选）你的“2分钟 <10% 变化自动休眠”的检测 ——
// 若已集成 IdleDetector_*，可在 Update 里调用 IdleDetector_Update(v_meas);

// 乘上放大倍数
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

    float v_raw = read_vin(METER_MODE_VOLT, AVG_N);
    // LOGF("ticks=%u v=%d idle=%d\r\n", s_ms_ticks,(int)(v_raw*1000.0f+0.5f),idle_last_ms);

    float dv = v_raw;

    // 减缓微小的波动
    float v_tmp = Volt_From_DV(dv);    // 真实输入（V，带正负号）
    // 正向零点漂移
    // v_tmp += VOLT_ZERO_OFFSET;
    
    // 实测线性标定
    float v_corrected = v_tmp * VOLT_CAL_GAIN + VOLT_CAL_OFFSET_V;
    // 本版本只支持正向测量
    // 开路残留约0.10V，低于门限直接归零
    if (v_corrected < VOLT_ZERO_DEADBAND_V)
    {
        v_corrected = 0.0f;
    }

    // 上限钳位，防止异常数据
    if (v_corrected > VOLT_MAX_V)
    {
        v_corrected = VOLT_MAX_V;
    }

    g_volt.last_v = v_corrected;
}
#pragma endregion

#pragma region 电流表业务逻辑
void AmpTask_Init(void)
{
    g_amp.iamp = 0.0f;
    g_amp.vin  = 0.0f;
    g_volt.next_ms = s_ms_ticks;
    LOGS("Amp init\r\n");
}
static float read_amp_vin(uint16_t *raw_out, int n)
{
    uint32_t acc = 0;

    for (int i = 0; i < n; ++i)
    {
        ADC_ScanOnce();
        acc += g_adc_pd2_raw;
    }

    uint16_t raw = (uint16_t)(acc / (uint32_t)n);

    if (raw_out != NULL)
    {
        *raw_out = raw;
    }

    return ADC_TO_V(raw);
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
    if (i >= 0) {
        // 正向误差：读数“偏高” → 需要减去一个正的幅值
        float e = _interp_err(iabs, ERR_POS_AT0_A, ERR_POS_ATFS_A, I_FULLSCALE_A);
        i += e*0.5f;
        if (i<0.f)
        {
            i=0.f;
        }
    } else {
        // 反向误差：读数“偏低”（更负） → 需要加回一个幅值
        float e = _interp_err(iabs, ERR_NEG_AT0_A, ERR_NEG_ATFS_A, I_FULLSCALE_A);
        i -= e*0.5f;
        if (i>0.f)
        {
            i=0.f;
        }
    }

    return i;
}

static void AmpTask_Update2(void)
{
    uint32_t now = s_ms_ticks;

    if ((int32_t)(now - g_volt.next_ms) < VOLT_SAMPLE_PERIOD_MS)
    {
        return;
    }

    g_volt.next_ms = now;

    // 旧版代码
    // g_amp.vin = read_vin(METER_MODE_AMP, AVG_N);

    // /*
    //  * 电流输入开路时，PD2实测会饱和到8190～8191。
    //  * 正常3A预计raw约4260，因此超过6000判为开路。
    //  */
    // if (g_adc_pd2_raw >= AMP_OPEN_RAW_TH)
    // {
    //     g_amp.iamp = 0.0f;
    //     return;
    // }

    // 用read_amp_vin去读电压顺便保存ADC值，方便判断开路
    uint16_t amp_raw = 0;
    g_amp.vin = read_amp_vin(&amp_raw, AVG_N);

    // 开路时实测8190～8191 此时显示电流为零
    if (amp_raw >= AMP_OPEN_RAW_TH)
    {
        g_amp.iamp = 0.0f;
        return;
    }

    // 实际电流达到约2.9A后，ADC停留在4050～4090
    // 统一按3A超量程处理
    if (amp_raw >= AMP_FULLSCALE_RAW_TH)
    {
        g_amp.iamp = I_FULLSCALE_A;
        return;
    }

    // 理论计算：0.1Ω采样电阻，放大倍数约10.1
    float i_raw = g_amp.vin / (0.1f * 10.1f);

    // 实测线性校准，根据207mA～2005mA实测数据校准
    float i_corrected =
        i_raw * AMP_CAL_GAIN + AMP_CAL_OFFSET_A;

    // 小电流及负值归零，单向正电流测量，低于零点死区统一显示0
    if (i_corrected < AMP_ZERO_DEADBAND_A)
    {
        i_corrected = 0.0f;
    }
    // 满量程限制，防止异常数据
    if (i_corrected > I_FULLSCALE_A)
    {
        i_corrected = I_FULLSCALE_A;
    }

    g_amp.iamp = i_corrected;
}
#pragma endregion

#pragma region 欧姆表业务逻辑
static float read_ohm_vin(uint16_t *raw_out, int n)
{
    uint32_t acc = 0;

    for (int i = 0; i < n; ++i)
    {
        ADC_ScanOnce();
        acc += g_adc_pa1_raw;
    }

    uint16_t raw = (uint16_t)(acc / (uint32_t)n);

    if (raw_out != NULL)
    {
        *raw_out = raw;
    }

    return ADC_TO_V(raw);
}
// Rx = 51 × ADC / (4029 - ADC)
static inline float compute_rx_ohm(uint16_t raw)
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
    if (rx < 30.0f)
    {
        rx = rx * 0.290f + 3.68f;
    }
    else
    {
        // 50～500Ω目前整体偏高约3%～6%
        rx *= 0.965f;
    }

    return rx;
}
static inline float compute_rx_kohm(uint16_t raw)
{
    if (raw >= 4000U)
    {
        return OHM_OPEN_VALUE;
    }

    if (raw <= 37U)
    {
        return 0.0f;
    }

    return 5086.3f * ((float)raw - 37.2f)
         / (4029.0f - (float)raw);
}

static inline float compute_rx_mohm(uint16_t raw)
{
    if (raw >= 3500U)
    {
        return OHM_OPEN_VALUE;
    }

    if (raw <= 55U)
    {
        return 0.0f;
    }

    return 454700.0f * ((float)raw - 55.0f)
         / (3686.0f - (float)raw);
}
static void set_range_pins(ohm_range_t r)
{
    switch (r)
    {
        case RANGE_OHM:
            GPIO_ResetBits(GPIOA, GPIO_Pin_2); // Ω档导通 此时是 51
            GPIO_SetBits(GPIOA, GPIO_Pin_3);
            GPIO_SetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);
            break;
        case RANGE_KOHM:
            GPIO_SetBits(GPIOA, GPIO_Pin_2);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3); // kΩ档导通 此时是 5.1k
            GPIO_SetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);
            break;
        case RANGE_MOHM:
            GPIO_SetBits(GPIOA, GPIO_Pin_2);
            GPIO_SetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(MOHM_SEL_PIN_PORT, MOHM_SEL_PIN_NUM);// 510k
            break;
    }
}

void OhmTask_Init(void)
{
    // 51MΩ
    g_ohm.rx_display = OHM_OPEN_VALUE;
    g_volt.next_ms = s_ms_ticks;

    OhmCtrl_GPIO_Init();
    
    g_ohm.range = RANGE_MOHM;
    g_ohm.st = OHM_S_SELECT_RANGE;   // ← 补上这行
    set_range_pins(g_ohm.range);
    LOGS("Ohm init MΩ\r\n");
    // 测试代码：锁定在k欧档测试一下510和51k
    g_ohm.st = OHM_S_MEASURE;

    // 重置LCD刷新的计时器，防止初始化时刷新屏幕
    g_lcd_buf.last_update_ms = g_volt.next_ms + VOLT_SAMPLE_PERIOD_MS;
}
// 根据档位自动计算电阻值 
static inline float compute_rx_by_range(
    ohm_range_t range,
    uint16_t raw)
{
    switch (range)
    {
        case RANGE_OHM:
            return compute_rx_ohm(raw);

        case RANGE_KOHM:
            return compute_rx_kohm(raw);

        case RANGE_MOHM:
            return compute_rx_mohm(raw);

        default:
            return 0.0f;
    }
}
// 每次调用仅推进一步；无阻塞、无 while(1)
void OhmTask_Update(void)
{   
    // 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((uint32_t)(now - g_volt.next_ms) < VOLT_SAMPLE_PERIOD_MS) return;
    g_volt.next_ms = now;

    // 档位选择
    if (g_ohm.st == OHM_S_SELECT_RANGE && false) {
        if (g_ohm.range != RANGE_KOHM)
        {
            g_ohm.range = RANGE_KOHM;
            set_range_pins(g_ohm.range);
        }
        // 用这个公式的话就是锁死 3V 参考电压去换算
        g_ohm.vin = read_ohm_vin(&g_ohm.raw, AVG_N);
#if 0
        // 这里也可以用 ADC 原始值来判断，vin 参考电压3V受5M分压影响可能是2.97V
        // kΩ档 测量 510Ω ADC=116 换算电压大约0.085 切Ω档
        if (g_ohm.vin<0.1f)
        {
            g_ohm.range = RANGE_OHM;
            set_range_pins(g_ohm.range);
            // 跳过这一次刷新LCD，防止换挡时读到低电阻错误地报警
            // g_ohm.rx_display = 500;//保留值
            g_lcd_buf.last_update_ms = now + LCD_UPDATE_MS;
        }
        // kΩ档 测量 51kΩ以上的电阻 ADC=2010 大约1.475V 切MΩ档
        else if (g_ohm.vin>1.46f)
        {
            g_ohm.range = RANGE_MOHM;
            set_range_pins(g_ohm.range);
        }
        // 测量完成 切到测量状态
        g_ohm.st = OHM_S_MEASURE;
#endif
        
        if (g_ohm.raw < KOHM_TO_OHM_RAW)
        {
            g_ohm.range = RANGE_OHM;
            set_range_pins(g_ohm.range);
            g_lcd_buf.last_update_ms = now + LCD_UPDATE_MS;
        }
        else if (g_ohm.raw > KOHM_TO_MOHM_RAW)
        {
            g_ohm.range = RANGE_MOHM;
            set_range_pins(g_ohm.range);
            g_lcd_buf.last_update_ms = now + LCD_UPDATE_MS;
        }

        g_ohm.st = OHM_S_MEASURE;
        
    } else {
        g_ohm.vin = read_ohm_vin(&g_ohm.raw, AVG_N);//read_vin(METER_MODE_OHM, AVG_N);

        float rx = compute_rx_by_range(g_ohm.range, g_ohm.raw);
        // M欧档退回千欧档、欧姆档切到千欧档、千欧档切到兆欧和欧姆档的逻辑
        // switch (g_ohm.range)
        // {
        //     case RANGE_OHM:
        //         // 510Ω实测ADC约3669，超过3700重新选档
        //         if (g_ohm.raw > OHM_TO_SELECT_RAW)
        //         {
        //             g_ohm.st = OHM_S_SELECT_RANGE;
        //         }
        //         break;

        //     case RANGE_KOHM:
        //         if (g_ohm.raw < KOHM_TO_OHM_RAW ||
        //             g_ohm.raw > KOHM_TO_MOHM_RAW)
        //         {
        //             g_ohm.st = OHM_S_SELECT_RANGE;
        //         }
        //         break;

        //     case RANGE_MOHM:
        //         // 51kΩ在M档约ADC420
        //         // 低于300说明应回kΩ档重新判断
        //         if (g_ohm.raw < MOHM_TO_SELECT_RAW)
        //         {
        //             g_ohm.st = OHM_S_SELECT_RANGE;
        //         }
        //         break;

        //     default:
        //         break;
        // }

        if (g_ohm.st == OHM_S_SELECT_RANGE)
        {
            g_lcd_buf.last_update_ms = now + LCD_UPDATE_MS;
        }
        else
        {
            g_ohm.rx_display = rx;
        }
    }
}

#pragma endregion

#pragma region 万用表初始化
// 初始化万用表子模块蜂鸣器、LCD、电池电量检测等
void MultimeterInit()
{
    // 引脚配置成输入模式
    MeterADC_GPIO_Init();
    ADC_Driver();
    
    // 关闭欧姆档换挡的三个 mos 管
    OhmCtrl_GPIO_Init();
    Ohm_AllOff();
    if (!hadSetMultiMeterMode)
    {
        // 若未初始化，设置为默认电压表
        LOGS("Meter IO Init\r\n");

        meter_mode = METER_MODE_VOLT;   // 默认电压表
        hadSetMultiMeterMode = true;
    }

	BuzzerInit();

    LCDInit();
    //暂时忽略LCD

    // 唤醒/初始化后：复位空闲检测器 和 LCD显示缓冲区
    memset((void*)&g_idle, 0, sizeof(g_idle));
    memset((void*)&g_lcd_buf, 0, sizeof(g_lcd_buf));

    BatteryTask_Init();
    
    SwitchMeterMode(meter_mode);

    hadSetMultimeterInit = true;
}
// 切换万用表三种工作模式，分别是电压表、电流表、欧姆表
static void SwitchMeterMode(meter_mode_t new_mode)
{
    if (meter_mode == new_mode && hadSetMultimeterInit) return;

    // 清图标缓冲，避免残留
    g_lcd_buf.mA_overf_neg_A_V_O_kO = 0;
    g_lcd_buf.bat_25_50_75_100_MO &= ICON_BAT_BROAD;

    meter_mode = new_mode;
    
    switch (meter_mode)
    {
        case METER_MODE_VOLT:
            // 先关闭欧姆相关硬件，避免串扰
            Ohm_AllOff();
            // PD3 高 PD4 低
            GPIO_SetBits(GPIOD, GPIO_Pin_3);
            GPIO_ResetBits(GPIOD, GPIO_Pin_4);
            VoltTask_Init();
            break;
        case METER_MODE_AMP:
            GPIO_SetBits(GPIOD, GPIO_Pin_3);
            GPIO_ResetBits(GPIOD, GPIO_Pin_4);
            AmpTask_Init();
            break;
        case METER_MODE_OHM:
            // 欧姆表 PD3 低 PD4 高
            GPIO_ResetBits(GPIOD, GPIO_Pin_3);
            GPIO_SetBits(GPIOD, GPIO_Pin_4);
            OhmTask_Init();
            break;
    }

    // 短按提示音
    // PWM_Cmd(TIM1, ENABLE);
    // delay_ms(25);
    // PWM_Cmd(TIM1, DISABLE);
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
欧姆表短路零点：PA1 ≈ 0.20V
Ω档最小有效分辨范围：粗略估计 5Ω~10Ω 以上才有意义
Ω档短路判断阈值：PA1 ≤ 0.23V 可先视为 0Ω/短路

短路需要交给 Ω 档判断。
开路/高阻需要交给 MΩ 档判断。
旧的 3.0V/5.049k 理想公式不适合，需要按实测零点和等效内阻标定。

Ω档参考电阻：约 51Ω
Ω档有效公式：Rx = 51 × V / (3.0 - V)
100Ω 实测：PA1≈1.96V，计算≈96Ω，基本通过
短路零点：PA1≈0.20~0.29V，等效约 4~6Ω，需要做短路阈值
开路：PA1≈2.8V，接近上限，应切到 kΩ档

*/

int main (void)
{
    first_init();
    
    #if ENABLE_LOG
        // uart0_tx 串口日志 PD5 uart1_tx 串口日志 PB1
        UART_Driver();
        LOGS("UART Init");
    #endif
    // 单独测试 IO 引脚
#if 0
    GPIO_InitTypeDef gi;

    // PA2 控制 Ω档
    gi.GPIO_Mode = GPIO_Mode_OutPP;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    gi.GPIO_Pin  = GPIO_Pin_2;
    GPIO_Init(GPIOA, &gi);

    // PA3 控制 kΩ档
    gi.GPIO_Pin  = GPIO_Pin_3;
    GPIO_Init(GPIOA, &gi);

    // PB1 控制 MΩ档 // 改成PD6试一下
    gi.GPIO_Pin  = GPIO_Pin_1;
    GPIO_Init(GPIOB, &gi);

    gi.GPIO_Pin  = GPIO_Pin_6;
    GPIO_Init(GPIOD, &gi);

    // 初始化 PD3 PD4 引脚，用于切换电压表、电流表、欧姆表
    gi.GPIO_Pin  = GPIO_Pin_3;
    GPIO_Init(GPIOD, &gi);
    gi.GPIO_Pin  = GPIO_Pin_4;
    GPIO_Init(GPIOD, &gi);

    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
    GPIO_SetBits(GPIOD, GPIO_Pin_6);//改用 D6 控制换挡，换挡逻辑优化

    GPIO_ResetBits(GPIOD, GPIO_Pin_3);
    GPIO_SetBits(GPIOD, GPIO_Pin_4);

    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    GPIO_ResetBits(GPIOD, GPIO_Pin_6);//改用 D6 控制换挡，换挡逻辑优化
    while (1)
    {
        /* code */
    }
#endif
    
#if 0
    // 测试兆欧功能
    meter_mode = METER_MODE_OHM;   // 欧姆表
    hadSetMultiMeterMode = true;
    MultimeterInit();
    while (1)
    {
        // 测试电压表
        // float v_pa1 = read_vin(METER_MODE_VOLT, AVG_N);
        // 分压公式计算
        // float v_in  = v_pa1 * 4.4f;
        // LOGF("PA1_raw:%d, PA1_mV:%d, VIN_mV:%d\r\n",
        //     (int)g_adc_pa1_raw,
        //     (int)(v_pa1 * 1000.0f + 0.5f),
        //     (int)(v_in * 1000.0f + 0.5f));
        
        // VoltTask_Update();
        
        // LOGF("g_volt.last_v:%d\r\n", (int)(g_volt.last_v*1000.0f));

        // 测试电流表
        // g_amp.vin = read_vin(METER_MODE_AMP,AVG_N);
        // 测试通过，可以正常打印ADC值
        // float i_calc = g_amp.vin / (0.1f * 10.1f);
        // LOGF("PD2_raw:%d, PD2_mV:%d, I_mA:%d\r\n",
        //     (int)g_adc_pd2_raw,
        //     (int)(g_amp.vin * 1000.0f + 0.5f),
        //     (int)(i_calc * 1000.0f + 0.5f));

        // 测试欧姆表
        g_ohm.vin = read_vin(METER_MODE_OHM,AVG_N);
        LOGF("OHM_PA1_raw:%d, OHM_mV:%d, RX:%d\r\n",
            (int)g_adc_pa1_raw,
            (int)(g_ohm.vin * 1000.0f + 0.5f));//OK

        // OhmTask_Update();

        // LOGF("range:%d, PA1_raw:%d, OHM_mV:%d, RX:%d\r\n",
        //     (int)g_ohm.range,
        //     (int)g_adc_pa1_raw,
        //     (int)(g_ohm.vin * 1000.0f + 0.5f),
        //     (int)(g_ohm.rx_display + 0.5f));

        // 测试电池电量
        // BatteryTask_Update();

        // 打印ADC扫描通道的ADC读数的原始值 PA1,PD2,PD3,PC4 
        // LOGF("PA1:%d,PD2:%d,PD3:%d,PC4:%d,V_REF:%d\r\n",
        //      (int)(g_adc_pa1_raw), 
        //      (int)(g_adc_pd2_raw), 
        //      (int)(g_adc_pd3_raw), 
        //      (int)(g_adc_pc4_raw),
        //      (int)(V_REF*1000.0f)
        // );

        // 测试LCD
        // LCD_Show_digits((int)(g_volt.last_v*1000.0f), 2);
        // 刷新LCD显示
        // LCD_DISPLAY_UPDATE();
        
        delay_ms(1000);
    }
#endif
    // 开机立刻休眠
    // deep_sleep();
    for (;;)
    {
        if (g_run_mode == RUN_MODE_NORMALWORK)
        {
            if(!hadSetMultimeterInit)
            {
                MultimeterInit();
            }
            // 处理短按切换电流表、欧姆表的事件
            if (g_short_press_event)
            {
                g_short_press_event = 0;

                meter_mode_t next = meter_mode;
                if (meter_mode == METER_MODE_VOLT) next = METER_MODE_AMP;
                else if (meter_mode == METER_MODE_AMP) next = METER_MODE_OHM;
                else next = METER_MODE_VOLT;

                SwitchMeterMode(next);
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


