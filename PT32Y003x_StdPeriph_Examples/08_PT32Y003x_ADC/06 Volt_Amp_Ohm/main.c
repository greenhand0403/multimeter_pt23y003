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

void Software_Delay(void)//软件延时
{
	u8 i, j,x;
	for(i=0; i<200; i++)
		for(j=0; j<200; j++)
			for(x=0; x<50; x++);
}
#pragma region 宏定义、全局变量和工具函数配置
char log_buffer[64];  // 用于打印日志 足够存储格式化字符串

// 读到的ADC原始数据 本来是全局的，给判断变化率10%使用的，但目前未用上
static uint16_t g_adc_pa1_raw = 0;  // 序号0（PA1）
static uint16_t g_adc_pc4_raw = 0;  // 序号1（PC4）
// 电阻表档位自动切换时启用 PA2 PA3 的输出功能
bool PA2_PA3_OutputEnabled = true;
// 万用表初始模式已设置
static bool hadSetMultiMeterMode = false;
// 万用表工作需要的外设已配置
static bool hadSetMultimeterInit = false;
// 运行模式：工作态 唤醒态 等待态
typedef enum { RUN_MODE_NORMALWORK = 0, RUN_MODE_DEEPSLEEP = 1, RUN_MODE_WAKEUP = 2} run_mode_t;
// 关机请求
volatile uint8_t poweroff_request = 0;

volatile uint8_t g_run_mode = RUN_MODE_NORMALWORK;   // 默认处于休眠模式
// Idle 监控（120s自动休眠功能）
static uint32_t idle_last_ms = 0;
static float last_value   = 0.0f;
#define IDLE_WINDOW_MS   (60000U)   // 60 s
#define CHANGE_THRESHOLD (0.10f)     // 10%
// 长按按键初始化和按键唤醒
// 可调参数
const uint32_t PWR_DEBOUNCE_MS = 200U;
const uint32_t PWR_LONGPRESS_MS = 2000U;
const uint32_t POSTWAKE_LONGPRESS_TIMEOUT = 5000U;

// 由 TIM2 每 10ms 扫描用到的计数
volatile uint16_t s_pwr_stable_ticks = 0;
volatile uint16_t s_pwr_press_ticks  = 0;
volatile uint8_t  s_pwr_last_sample  = 1;   // 1=未按, 0=按下

// 万用表模式： 电压表 电流表 欧姆表
typedef enum { METER_MODE_VOLT = 0, METER_MODE_AMP = 1,METER_MODE_OHM = 2 } meter_mode_t;
static meter_mode_t meter_mode = 0;
// ===== 开机“1V偏置” =====
// 无负载时 PA1 的基准电压（零点，可标定）
static float   g_v1_ref   = 1.0000f;  // 开机测到的“1V”偏置（例如 0.9973）
static uint8_t g_v1_ready = 0;

// 统一的“去偏置”助手：把原始电压换算成相对 1V 的差值 代表第二级运放的输入电压 便于后续计算
static inline float V_DV(float v_raw) { return v_raw - g_v1_ref; }
// 重新记录空闲状态ADC采集电压变化<10%的起始时间
#define ADC_TO_V(x)   ((x) * 2.0f / 4095.0f)
/***** 配置与常量 *****/
#define AVG_N            5

// 硬件与标定数值
#define RSHUNT           0.1f      // 采样电阻
#define I_IDLE_A         0.001f    // <1mA 视为无负载

// A 档（默认）
#define GAIN_A           4.0f

// mA 档（带你的斜率修正系数）
#define GAIN_mA          33.99f
#define MA_SLOPE_FIX     9.2863f   // 你前面标定得出的斜率系数

// mA 档上限（=294mA）
#define I_MA_MAX         0.294f

#define ZERO_BAND_V       0.0020f                    // 零点死区：|ΔV|<2mV 视为0V
// 你当前用的“斜率校准系数”，把ΔV（单位V）映射成表笔端电压（单位V）
#define VOLT_SCALE        (10000.0f/379.2f)          // ≈ 26.38191
// 量程与显示
#define V_POS_MAX         12.00f                     // 正向最大显示
#define V_NEG_MIN         -12.0f                     // 反向最小显示
#define OVERFLOW_MARGIN   0.05f                      // 超量程提前量（避免边界抖动）

#define VIN_OPEN_TH         1.920f     // ≥此电压视为开路/移除
#define VIN_ZERO_TH         0.0008f    // <此电压视为短路(0Ω)；比 0.001 更保守
// 分档电压门限（无交叉）：Ω < 0.0020V；kΩ < 0.1818V；MΩ < 1.92V
#define VIN_OHM_MAX         0.0020f
#define VIN_KOHM_MAX        0.1818f

// 分档迟滞（进入/退出不同阈值，抑制抖动）
#define VIN_OHM_ENTER       0.0018f
#define VIN_OHM_EXIT        0.0022f
#define VIN_K_ENTER         0.1700f
#define VIN_K_EXIT          0.1900f

// 采样电阻（含你之前微调可继续放在这里统一管理）
#define RS_OHM_RAW          50.9949f     // 51Ω
#define RS_KOHM_RAW         5049.505f     // 5.1kΩ
#define RS_MOHM_RAW         510000.0f     // 510kΩ

// 分档校准（斜率/零点），后续实测再填；默认1与0表示未校准
#define GAIN_OHM            1.0000f
#define GAIN_KOHM           0.9720f      // 你之前估过 kΩ 偏高，默认给个缩小系数
#define GAIN_MOHM           0.9930f      // 你之前估过 MΩ 偏高
#define OFFS_OHM            0.0f
#define OFFS_KOHM           0.0f
#define OFFS_MOHM           0.0f
// 电阻表档位
typedef enum { RANGE_OHM = 0, RANGE_KOHM, RANGE_MOHM } ohm_range_t;
#pragma endregion
#pragma region 串口驱动
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
	// GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PB1 TX1
	// GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PD1 RX1
	GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0,ENABLE);	//PD5 TX1
	GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0,ENABLE);	//PD6 RX1
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
	// UART_Init(UART1, &UART_InitStruct);

	/*开启UART1的收发功能*/
	// UART_Cmd(UART1, ENABLE);
	UART_Init(UART0, &UART_InitStruct);

	/*开启UART0的收发功能*/
	UART_Cmd(UART0, ENABLE);
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
        // UART_SendData(UART1, *str++);
        // while (UART_GetFlagStatus(UART1, UART_FLAG_TXE) == RESET);
		UART_SendData(UART0, *str++);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TXE) == RESET);
    }
}
#pragma endregion
#pragma region 输出参考电压2V并配置PA1的ADC采集
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
	ADC_InitTypeDef  ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_Prescaler = 48;						 	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Single;						//单次转换模式
	ADC_InitStruct.ADC_TriggerSource = ADC_TriggerSource_Software;
	ADC_InitStruct.ADC_TimerTriggerSource=ADC_TimerTriggerSource_TIM1ADC;//定时源触发选择TIM0事件
	ADC_InitStruct.ADC_Align = ADC_Align_Left;					//左对齐
	ADC_InitStruct.ADC_Channel = ADC_Channel_1;//PA1
	ADC_InitStruct.ADC_ReferencePositive = ADC_ReferencePositive_BG2v0;
	ADC_InitStruct.ADC_BGVoltage=ADC_BGVoltage_BG1v0;//BGS电压1.0v
	ADC_BGCRSetBGNC(ADC);// SET ADC_BGNC BIT
	ADC_Init(ADC, &ADC_InitStruct);

    // ★ 扫描序列：序号0=PA1(ADC1)【测量端】，序号1=PC4(ADC7)【电池】
    ADC_ScanChannelConfig(ADC, ADC_Channel_1, 0);
    ADC_ScanChannelConfig(ADC, ADC_Channel_7, 1);
    ADC_ScanChannelNumberConfig(ADC, 2);
    ADC_ScanCmd(ADC, ENABLE);

    // （可选）硬件平均
    ADC_AverageTimesConfig(ADC, ADC_AverageTimes_16);

    ADC_Cmd(ADC, ENABLE);
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_RDY));
}
#pragma endregion
#pragma region 自动休眠逻辑
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
    TIM_ClearFlag(TIM2, TIM_FLAG_ARF);
    TIM_ITConfig(TIM2, TIM_IT_ARI, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM2, ENABLE);
}
// 配置 PC5 的外部中断，用于休眠唤醒功能
void Wake_Key_Init(void)
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
// 进入休眠状态的逻辑，关闭外设
void deep_sleep()
{
    if(g_run_mode == RUN_MODE_NORMALWORK)
    {
        // 休眠提示音
        PWM_Cmd(TIM1, ENABLE);
        delay_ms(200);
        PWM_Cmd(TIM1, DISABLE);

        sprintf(log_buffer, "DEEPSLEEP ms_ticks=%u\r\n", s_ms_ticks);
        UART1_SendString(log_buffer);
        // 等待PC5按键松开
        while (GPIO_ReadDataBit(GPIOC,GPIO_Pin_5)==0)
        {
            delay_ms(20);
        }
        
        // 关闭TIM2长按按键的计时器
        TIM_ClearFlag(TIM2, TIM_FLAG_ARF);
        TIM_ITConfig(TIM2, TIM_IT_ARI, DISABLE);
        NVIC_DisableIRQ(TIM2_IRQn);	
        TIM_Cmd(TIM2, DISABLE);
        
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
        // 关闭 UART 等 外设 现在用 UART0 做 log
        UART_Cmd(UART0, DISABLE);
        // UART_Cmd(UART1, DISABLE);
        
        // 打开外部中断 配置PC5为唤醒源
        Wake_Key_Init();
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // 进睡前关闭系统定时器
        g_run_mode = RUN_MODE_DEEPSLEEP;
        poweroff_request = 0;
        // EXTI_ClearFlag(EXTIC, GPIO_Pin_5);
        // EXTI_ClearFlag(EXTIC, GPIO_Pin_5);
        // 进入深度睡眠
        PWR_EnterDeepSleepMode(PWR_DeepSleepEntry_WFI);
        g_run_mode = RUN_MODE_WAKEUP;
        // —— 从 EXTI 唤醒返回 —— 关闭唤醒用 EXTI，避免运行态乱中断
        Wake_Key_EXITDisable();

        // 恢复运行态外设（最少：SysTick 用于5秒自动休眠 + TIM2 用于长按2秒进入工作态）
        SysTick_Init_1kHz();// 重新建立 1ms 节拍
        // 配置上拉按键 PC5
        PowerKey_GPIO_Init();
        TIM2_Init_10ms(); // 重启 10ms 键扫描
        // 清掉键计数，避免“带电平穿越”造成误判
        PowerKey_ResetCounters();
        
        // 记录时间戳
        uint32_t t0 = s_ms_ticks;

        UART_Driver();
        UART_Cmd(UART0, ENABLE);

        sprintf(log_buffer,"WAKEUP ticks=%u\r\n",s_ms_ticks);
        UART1_SendString(log_buffer);

        // 在唤醒后保持按住 2 秒回到工作态 若未长按 5s后重新睡眠 由TIM2中断服务程序修改系统运行状态
        while(g_run_mode == RUN_MODE_WAKEUP)
        {
            // 到时未确认 -> 回睡 5000ms即5秒
            if ((s_ms_ticks - t0) >= POSTWAKE_LONGPRESS_TIMEOUT) {
                // 先关TIM2防抖动，防止被打断
                TIM_ClearFlag(TIM2, TIM_FLAG_ARF);
                TIM_ITConfig(TIM2, TIM_IT_ARI, DISABLE);
                NVIC_DisableIRQ(TIM2_IRQn);	
                TIM_Cmd(TIM2, DISABLE);
                // 再次入睡（递归进入OK）
                g_run_mode = RUN_MODE_NORMALWORK;
                UART1_SendString("sleep again\r\n");
                deep_sleep();
                return; // 不会走到这里
            }
        }

        idle_last_ms = s_ms_ticks;//重置变化率<10%的120s计数
        sprintf(log_buffer,"NORMALWORK ms_ticks=%u\r\n",s_ms_ticks);
        UART1_SendString(log_buffer);
        // 返回正常工作，清除标志位，此时需要再次调用万用表外设配置函数
        hadSetMultimeterInit = false;
    }
}
// 每次有新测量值（电压/电流/电阻任选一个“代表值”）都调用一次
static void Idle_Update(float cur_value)
{
    // ★ 防回拨：若 s_ms_ticks 被清零/回拨，使 idle_last_ms 对齐当前
    if ((int32_t)(s_ms_ticks - idle_last_ms) < 0) {
        idle_last_ms = s_ms_ticks;
    }
    
    float base = (last_value == 0.0f) ? 1.0f : last_value;
    float diff = (cur_value - last_value) / base;
    if (diff < 0) diff = -diff;

    if (diff >= CHANGE_THRESHOLD) {
        // 清除空闲时间时间戳
        idle_last_ms = s_ms_ticks;// 有明显变化 → 视为活跃
    }
    last_value = cur_value;
    // 当前时间 - 上一次空闲时间戳
    if ((s_ms_ticks - idle_last_ms) >= IDLE_WINDOW_MS) {
        sprintf(log_buffer,"%d: %d - %d\r\n",g_run_mode,s_ms_ticks,idle_last_ms);
        UART1_SendString(log_buffer);
        // idle_last_ms = 0;
        // s_ms_ticks = 0;
        if(g_run_mode == RUN_MODE_NORMALWORK) {
            deep_sleep();
        }// 进入深睡
    }
}
#pragma endregion
#pragma region 万用表模式选择 PA2 PA3 输出配置
// init multi meter io PA2 PA3
void MultiMeterIOInit(void)
{
	if (PA2_PA3_OutputEnabled)
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

		PA2_PA3_OutputEnabled = false;
	}
}
void MultiMeterIOOutputEnable(void)
{
	if (!PA2_PA3_OutputEnabled)
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

		PA2_PA3_OutputEnabled = true;
	}
}
#pragma endregion
#pragma region 万用表初始化设置
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

// 将开机测到的初始电压“记为1V”——存入 g_v1_ref
static void CaptureInitialV1(uint16_t samples)
{
    // 可丢弃几次读数让 ADC 稳定
    for (int i = 0; i < 4; ++i) (void)read_vin(1);

    uint16_t N = samples ? samples : 64;
    float sum = 0.f;
    for (uint16_t i = 0; i < N; ++i) sum += read_vin(1);
    float v = sum / (float)N;

    // 简单边界保护：若读数离谱，则退回 1.0000
    if (v < 0.80f || v > 1.20f) v = 1.0000f;

    g_v1_ref   = v;
    g_v1_ready = 1;
}

// MΩ 档对 Rs 的微调（按你此前经验：低/中/高阻做轻微补偿，可选）
static inline float tune_Rs_Mohm(float vin)
{
    float Rs = RS_MOHM_RAW;
    if (vin > 1.34f)      Rs = RS_MOHM_RAW * 0.992f;   // 高阻
    else if (vin < 0.76f) Rs = RS_MOHM_RAW * 1.00503f; // 低阻
    return Rs;
}

static inline float compute_rx(float vin, float Rs)
{
    // 经典电阻表：Rx = Rs * Vin / (Vref - Vin)，Vref=2.0V
    return Rs * vin / (2.0f - vin);
}

static void set_range_pins(ohm_range_t r)
{
    MultiMeterIOOutputEnable(); // 需要控制PA2/PA3时转为输出
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
#pragma endregion
#pragma region 液晶屏初始化
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
	// TODO: 测试LCD屏
    // uint8_t seg_data[] = {DISPLAY_HALF_BAT};  // 示例数据
    // HT1621_WriteData(HALF_BAT_ADDR, seg_data, 1);
    // , DISPLAY_HZ, DISPLAY_MOhm
	// HT1621_WriteData(HALF_BAT_ADDR, DISPLAY_HALF_BAT, 1);
	// HT1621_WriteData(HALF_BAT_ADDR, DISPLAY_HZ, 1);
	// HT1621_WriteData(HALF_BAT_ADDR, DISPLAY_MOhm, 1);
}
static void lcd_show_ready(void)           { /* LCD_ShowReady(); */ }
static void lcd_show_overflow(ohm_range_t r){ /* LCD_ShowOverflow(r); */ }
static void lcd_show_ohms(float rx, ohm_range_t r)
{
    // 这里只做串口示例，LCD 你自己接
    switch (r)
    {
        case RANGE_OHM:  sprintf(log_buffer, "Ohm: %.0f \r\n", rx); break;          // 0000~0999Ω
        case RANGE_KOHM: sprintf(log_buffer, "Ohm: %.2f k\r\n", rx/1000.0f); break; // xx.xx kΩ
        case RANGE_MOHM: sprintf(log_buffer, "Ohm: %.2f M\r\n", rx/1e6f); break;    // xx.xx MΩ
    }
    UART1_SendString(log_buffer);
}
#pragma endregion
#pragma region 电池电量检测
// === 你可以按芯片手册调整这一行 ===
#define BATT_ADC_CHANNEL      ADC_Channel_7   // ★ PC4 对应的 ADC 通道（若不对，请改）
#define BATT_SAMPLE_PERIOD_MS 1000U           // ★ 每秒一次
#define BATT_SAMPLES_N        16              // ★ 每次平均 16 点
#define BATT_ALPHA            0.3f            // ★ 简单一阶滤波系数（0.0~1.0）
// ★ 电量分档阈值（按你给的门限，单位：V，针对PC4测得的电压）
#define BATT_TH_4             1.55f
#define BATT_TH_3             1.40f
#define BATT_TH_2             1.25f
#define BATT_TH_1             1.15f

static struct {
    uint32_t next_ms;
    float    v_filt;   // 低通后的电压
    int      level;    // 0..4
} g_batt;

// ★ PC4 接 ADC：打开模拟复用 + 输入无上下拉
static void Battery_GPIO_Init(void)
{
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_4, ENABLE);  // PC4→ADC
    GPIO_InitTypeDef gi;
    gi.GPIO_Mode = GPIO_Mode_In;
    gi.GPIO_Pin  = GPIO_Pin_4;
    gi.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOC, &gi);
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
    Battery_GPIO_Init();
    g_batt.next_ms = s_ms_ticks;
    g_batt.v_filt  = -1.0f;  // 标识首样本
    g_batt.level   = -1;     // 未定级
}

void BatteryTask_Update(void)
{
    if ((int32_t)(s_ms_ticks - g_batt.next_ms) < 0) return;
    g_batt.next_ms = s_ms_ticks + BATT_SAMPLE_PERIOD_MS;

    // 取 PC4 的电压；如需平均可循环 ADC_ScanOnce() 多次求平均
    ADC_ScanOnce();
    float v = ADC_TO_V(g_adc_pc4_raw);

    if (g_batt.v_filt < 0.0f) g_batt.v_filt = v;                // 首样本直赋
    else                      g_batt.v_filt = (1.0f-BATT_ALPHA)*g_batt.v_filt + BATT_ALPHA*v;

    int lvl = Battery_LevelFromV(g_batt.v_filt);
    // 打印（你后续可把 lvl 映射到 LCD 图标）
    sprintf(log_buffer, "BATT: %.3fV [%d/4]\r\n", g_batt.v_filt, lvl);
    UART1_SendString(log_buffer);

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
	PWM_TimeBaseInitType.PWM_AutoReloadValue = 369;
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

    /* 初始化PWM输出 此处报错了*/
    PWM_OCInit(TIM1, &OutInit);
}

#pragma endregion
#pragma region 电压表业务逻辑
// === 采样与显示参数（按需调整） ===
#ifndef VOLT_SAMPLE_PERIOD_MS
#define VOLT_SAMPLE_PERIOD_MS   1000U    // 电压更新周期：20ms
#endif

#ifndef K_VOLT_SLOPE
// 把“相对1V的差值(dv)”换算成输入端电压（单位: V）
// 例：若你实测 9V -> dv≈0.682V，则 K≈9/0.682 ≈ 13.200
#define K_VOLT_SLOPE            (10000.0f/379.2f)
#endif

// 是否做上/下限钳位（例如 0~12V）
#ifndef VOLT_MAX_V
#define VOLT_MAX_V              12.0f
#endif
#ifndef VOLT_MIN_V
#define VOLT_MIN_V              (-12.0f)
#endif

// ——（可选）你的“2分钟 <10% 变化自动休眠”的检测 ——
// 若已集成 IdleDetector_*，可在 Update 里调用 IdleDetector_Update(v_meas);

// 小工具：把 dv -> 物理电压（V）
static inline float Volt_From_DV(float dv) {
    return dv * K_VOLT_SLOPE;
}

typedef struct {
    uint32_t next_ms;   // 下次允许采样的时间戳(ms)
    float    last_v;    // 上一帧电压，供抖动/保留显示使用（可选）
} volt_ctx_t;

static volt_ctx_t g_volt;

/* 供你替换为真实的显示函数
   正电压：保留2位小数；负电压：保留1位小数；越界时显示上/下限 + 溢出标志 */
static void lcd_show_voltage_pos(float v, int overflow)
{
    char buf[32];
    if (overflow) {
        // 上限溢出（例如显示固定 12.00 + 标志）
        sprintf(buf, "V=%.4fV(OVF)", VOLT_MAX_V);
    } else {
        sprintf(buf, "V=%.4fV", v);
    }
    UART1_SendString(buf); UART1_SendString("\r\n");
}
static void lcd_show_voltage_neg(float v, int overflow)
{
    char buf[32];
    if (overflow) {
        sprintf(buf, "V=-%.3fV(OVF)", -VOLT_MIN_V);
    } else {
        sprintf(buf, "V=%.3fV", v);   // v为负
    }
    UART1_SendString(buf); UART1_SendString("\r\n");
}

/* ========== 初始化：不重采“1V基准”，使用开机时的 g_v1_ref ========== */
void VoltTask_Init(void)
{
    g_volt.next_ms = s_ms_ticks;  // 立即可以更新
    g_volt.last_v  = 0.0f;
}

/* ========== 单步更新：无阻塞、无死循环 ========== */
void VoltTask_Update(void)
{
    // 1) 节流：到点再测
    uint32_t now = s_ms_ticks;
    if ((int32_t)(now - g_volt.next_ms) < 0) return;
    g_volt.next_ms = now + VOLT_SAMPLE_PERIOD_MS;

    // 2) 读取原始电压并去偏置
    float v_raw = read_vin(AVG_N);     // 你已有的平均读法

    sprintf(log_buffer,"ticks=%u v= %.4f idle=%d\r\n", s_ms_ticks,v_raw,idle_last_ms);
    UART1_SendString(log_buffer);

    // 对应换算公式是 ( vout - 0.9983 ) * 10000.0 / 379.2
    float dv = V_DV(v_raw);
    float v_in  = Volt_From_DV(dv);    // 真实输入（V，带正负号）

    // 3) 上/下限钳位与显示
    if (v_in >= 0.0f) {
        int ovf = (v_in > VOLT_MAX_V);
        float v_disp = ovf ? VOLT_MAX_V : v_in;
        lcd_show_voltage_pos(v_disp, ovf);
    } else {
        int ovf = (v_in < VOLT_MIN_V);
        float v_disp = ovf ? VOLT_MIN_V : v_in;
        lcd_show_voltage_neg(v_disp, ovf);
    }

    g_volt.last_v = v_in;
}
#pragma endregion
#pragma region 电流表业务逻辑
// ===== 电流表状态机 =====
typedef enum { AMP_S_IDLE_WAIT = 0, AMP_S_RANGE_DECIDE, AMP_S_MEASURE_A, AMP_S_MEASURE_mA } amp_state_t;

static struct {
    amp_state_t st;
    bool mAflag;     // false=A 档, true=mA 档
    float vin, iamp; // 最近一次的测量数据
} g_amp;
void AmpTask_Init(void)
{
    // PA2 作为量程控制：0=A档，1=mA档
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OutPP;
    GPIO_InitStruct.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, GPIO_Pin_2); // 默认 A 档
    g_amp.mAflag = false;
    g_amp.st     = AMP_S_IDLE_WAIT;

    UART1_SendString("Amp init: A-range (PA2=0)\r\n");
}
// 每次调用仅推进一步；无阻塞、无 while(1)
void AmpTask_Update(void)
{
    switch (g_amp.st)
    {
    case AMP_S_IDLE_WAIT:
        // 等待接入（按电流阈值更稳）
        g_amp.vin  = read_vin(AVG_N);
        g_amp.iamp = (V_DV(g_amp.vin) * 10.0f / GAIN_A);  // 以 A 档公式估计
        // 串口可选日志
        // sprintf(log_buffer, "[IDLE] I≈%.0fmA vin=%.4f\r\n", g_amp.iamp*1000.0f, g_amp.vin); UART1_SendString(log_buffer);
        if (g_amp.iamp >= I_IDLE_A) {
            g_amp.st = AMP_S_RANGE_DECIDE;
        }
        break;

    case AMP_S_RANGE_DECIDE:
        g_amp.vin = read_vin(AVG_N);
        // 由 I_MA_MAX 从 A 档推导 mA 进入门槛
        if (g_amp.vin < (g_v1_ref + (GAIN_A * I_MA_MAX * RSHUNT))) {       // 进入 mA 档
            GPIO_SetBits(GPIOA, GPIO_Pin_2);
            g_amp.mAflag = true;
            g_amp.st     = AMP_S_MEASURE_mA;
            UART1_SendString("enter mA range (PA2=1)\r\n");
        } else {                           // 留在 A 档
            GPIO_ResetBits(GPIOA, GPIO_Pin_2);
            g_amp.mAflag = false;
            g_amp.st     = AMP_S_MEASURE_A;
            UART1_SendString("stay in A range (PA2=0)\r\n");
        }
        break;

    case AMP_S_MEASURE_mA:
        g_amp.vin  = read_vin(AVG_N);
        g_amp.iamp = V_DV(g_amp.vin) * (MA_SLOPE_FIX / GAIN_mA);// A
        // 退出条件
        if (g_amp.iamp >= I_MA_MAX) {                // 超 mA 档上限 -> 重新判档
            UART1_SendString("mA->A (>=294mA)\r\n");
            g_amp.st = AMP_S_RANGE_DECIDE;
            break;
        }
        if (g_amp.iamp < I_IDLE_A) {                 // 无负载 -> 回等待
            UART1_SendString("load removed (mA)\r\n");
            g_amp.st = AMP_S_IDLE_WAIT;
            break;
        }

        // 显示（LCD 自接入）
        sprintf(log_buffer, "I=%.0fmA (vin=%.4f)\r\n", g_amp.iamp*1000.0f, g_amp.vin);
        UART1_SendString(log_buffer);
        break;

    case AMP_S_MEASURE_A:
        g_amp.vin  = read_vin(AVG_N);
        g_amp.iamp = V_DV(g_amp.vin) * 10.0f / GAIN_A; // A
        if (g_amp.iamp < I_IDLE_A) {                 // 无负载 -> 回等待
            UART1_SendString("load removed (A)\r\n");
            g_amp.st = AMP_S_IDLE_WAIT;
            break;
        }

        if (g_amp.iamp >= 2.501f) {
            UART1_SendString("OVER: >=2.501A\r\n");
        } else {
            sprintf(log_buffer, "I=%.3fA (vin=%.4f)\r\n", g_amp.iamp, g_amp.vin);
            UART1_SendString(log_buffer);
        }
        break;
    }
}
#pragma endregion
#pragma region 欧姆表业务逻辑
// ===== 欧姆表状态机 =====
typedef enum { OHM_S_WAIT_CONNECT = 0, OHM_S_SELECT_RANGE, OHM_S_MEASURE } ohm_state_t;

static struct {
    ohm_state_t st;
    ohm_range_t range;
    float vin;
} g_ohm;

void OhmTask_Init(void)
{
    g_ohm.st    = OHM_S_WAIT_CONNECT;
    g_ohm.range = RANGE_OHM; // 初值无所谓，进入 SELECT_RANGE 会重判
    UART1_SendString("Ohm init\r\n");
}

// 每次调用仅推进一步；无阻塞、无 while(1)
void OhmTask_Update(void)
{
    switch (g_ohm.st)
    {
    case OHM_S_WAIT_CONNECT:
        g_ohm.vin = read_vin(AVG_N);
        // 高于开路阈值 -> 继续等待
        if (g_ohm.vin > VIN_OPEN_TH) {
            // 可显示“— — — —”或 Ready
            // lcd_show_ready();
            return;
        }
        // 短路特判
        if (g_ohm.vin < VIN_ZERO_TH) {
            lcd_show_ohms(0.0f, RANGE_OHM);
            return; // 仍保持 WAIT，直到电压离开短路或被移除
        }
        g_ohm.st = OHM_S_SELECT_RANGE;
        break;

    case OHM_S_SELECT_RANGE:
        g_ohm.vin = read_vin(AVG_N);
        if (g_ohm.vin > VIN_OPEN_TH) { g_ohm.st = OHM_S_WAIT_CONNECT; return; }
        if (g_ohm.vin < VIN_OHM_ENTER) {
            g_ohm.range = RANGE_OHM;   set_range_pins(g_ohm.range); UART1_SendString("range: ohm\r\n");
        } else if (g_ohm.vin < VIN_K_ENTER) {
            g_ohm.range = RANGE_KOHM;  set_range_pins(g_ohm.range); UART1_SendString("range: k\r\n");
        } else {
            g_ohm.range = RANGE_MOHM;  set_range_pins(g_ohm.range); UART1_SendString("range: M\r\n");
        }
        g_ohm.st = OHM_S_MEASURE;
        break;

    case OHM_S_MEASURE: {
        g_ohm.vin = read_vin(AVG_N);

        // 拔掉/开路 -> 回等待
        if (g_ohm.vin >= VIN_OPEN_TH) { UART1_SendString("ohm: open/remove\r\n"); g_ohm.st = OHM_S_WAIT_CONNECT; return; }

        // 档位迟滞（防抖动）
        if (g_ohm.range == RANGE_OHM && g_ohm.vin > VIN_OHM_EXIT) { g_ohm.st = OHM_S_SELECT_RANGE; return; }
        if (g_ohm.range == RANGE_KOHM) {
            if (g_ohm.vin <= VIN_OHM_ENTER || g_ohm.vin > VIN_K_EXIT) { g_ohm.st = OHM_S_SELECT_RANGE; return; }
        }
        if (g_ohm.range == RANGE_MOHM && g_ohm.vin < VIN_K_ENTER) { g_ohm.st = OHM_S_SELECT_RANGE; return; }

        // 计算 Rx
        float Rs = (g_ohm.range == RANGE_OHM)  ? RS_OHM_RAW :
                   (g_ohm.range == RANGE_KOHM) ? RS_KOHM_RAW : tune_Rs_Mohm(g_ohm.vin);
        float rx = compute_rx(g_ohm.vin, Rs);

        // 分档校准
        if (g_ohm.range == RANGE_OHM)   rx = rx * GAIN_OHM  + OFFS_OHM;
        if (g_ohm.range == RANGE_KOHM)  rx = rx * GAIN_KOHM + OFFS_KOHM;
        if (g_ohm.range == RANGE_MOHM)  rx = rx * GAIN_MOHM + OFFS_MOHM;

        // 溢出与蜂鸣器示例（<51Ω响）
        bool overflow = false;
        if (g_ohm.range == RANGE_OHM   && rx >  999.9f)     overflow = true;
        if (g_ohm.range == RANGE_KOHM  && rx >  99990.0f)   overflow = true;
        if (g_ohm.range == RANGE_MOHM  && rx > 10000000.0f) overflow = true;

        if (g_ohm.range == RANGE_OHM) {
            // 蜂鸣器 PWM 时钟 TIM1
            if (rx < 51.0f && !(TIM1->CR1 & 1)) PWM_Cmd(TIM1, ENABLE);
            else if (rx >= 51.0f && (TIM1->CR1 & 1)) PWM_Cmd(TIM1, DISABLE);
        }

        if (overflow) {
            // lcd_show_overflow(g_ohm.range);
        } else {
            lcd_show_ohms(rx, g_ohm.range);
        }
        break;
    }
    }
}

#pragma endregion
#pragma region 万用表初始
void MultimeterInit()
{
    BuzzerInit();
    LCDInit();
    if (!hadSetMultiMeterMode)
    {
        UART1_SendString("Meter IO Init");
        // PA2 PA3 输入配置 根据情况选择电流表、电压表或欧姆表
        MultiMeterIOInit();
    }
	// PA1 和 PC5 作为 ADC 输入
    BatteryTask_Init();
	ADC_Driver();

    idle_last_ms = s_ms_ticks;//重置变化率<10%的120s计数
    // if (!hadSetMultiMeterMode)
    {
        // init state: PA2 PA3 , LOW LOW mean VoltTest, LOW HIGH mean AmpTest, HIGH HIGH mean OhmTest
        if (GPIO_ReadDataBit(GPIOA,GPIO_Pin_2)==RESET)
        {
            if (GPIO_ReadDataBit(GPIOA,GPIO_Pin_3)==RESET)
            {
                sprintf(log_buffer, "Volt Mode\r\n");
                UART1_SendString(log_buffer);
                meter_mode = METER_MODE_VOLT;
            }
            else
            {
                sprintf(log_buffer, "Amp Mode\r\n");
                UART1_SendString(log_buffer);
                meter_mode = METER_MODE_AMP;
            }
        }
        else
        {
            sprintf(log_buffer, "Ohm Mode\r\n");
            UART1_SendString(log_buffer);
            meter_mode = METER_MODE_OHM;
        }
        hadSetMultiMeterMode = true;
    }
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

    // ★ 新增：开机抓一次“1V偏置”
    CaptureInitialV1(100);
    sprintf(log_buffer,"Mode=%d ticks=%u v_ref=%.4f\r\n", meter_mode, s_ms_ticks, g_v1_ref);
    UART1_SendString(log_buffer);

    // 初始化仪表成功提示音
    PWM_Cmd(TIM1, ENABLE);
    delay_ms(200);
    PWM_Cmd(TIM1, DISABLE);
    
    hadSetMultimeterInit = true;
}
#pragma endregion
#pragma region 主循环逻辑

int main (void)
{
    SysTick_Init_1kHz();// 系统时钟定时器 us ms 计时已测试 准确
    // 测试 LCD
    LCDInit();
    while (1)
    {
        uint8_t ff[16]; for (int i=0;i<16;i++) ff[i]=0xFF;
        HT1621_WriteData(0x00, ff, 16);
        delay_ms(2000);
        HT1621_Clear();
        delay_ms(2000);
    }
    
    PowerKey_GPIO_Init(); // 长按开关机的按键输入配置
    TIM2_Init_10ms();// 长按时间定时器TIM2
    PowerKey_ResetCounters();// 长按时间计数清零

    // uart0_tx 串口日志 PD5
    UART_Driver();
    UART1_SendString("UART1 Init");

    // 最新改动，开机直接进休眠，长按2秒才会回
    deep_sleep();
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
                AmpTask_Update();
                break;
            case METER_MODE_OHM:
                OhmTask_Update();
                break;
            default:
                break;
            }
            // 变化率 <10% 自动休眠
            Idle_Update(g_adc_pa1_raw);
            BatteryTask_Update();     // ★ 每秒打印一次电池电量
            if (poweroff_request)//要求长按松手后才关机
            {
                UART1_SendString("wait to poweroff");
                deep_sleep();
            }
            delay_ms(20);
        }
        else if (g_run_mode == RUN_MODE_DEEPSLEEP)
        {
            // sprintf(log_buffer,"RUN_POSTWAKE ms_ticks=%u\r\n",s_ms_ticks);
            // UART1_SendString(log_buffer);
        }
        else if (g_run_mode == RUN_MODE_WAKEUP)
        {

        }
    }
    // 延迟打印看工作状态和运行时间
    // TODO: 电流表和电压表要改 加了个固定负号图标，不占用第一位数，表笔正反接时，除了负号变化，其他数字不变
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


