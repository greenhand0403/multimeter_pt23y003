// servo_driver.c
#include "servo_driver.h"

#define SERVO_TIMER_PSC 47 // prescaler -> 1 MHz tick (assuming 48MHz SYSCLK)
#define SERVO_TIMER_ARR 19999 // 20 ms period at 1MHz tick
// 恰好映射为500us~2500us 对应 0°~180° 舵机控制的角度范围
#define SERVO_MIN_PULSE 500 // 0 degree -> 500 us
#define SERVO_MAX_PULSE 2500 // 180 degree -> 2500 us

void servo_init(void)
{
	GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_4, AFIO_AF_2, ENABLE); 
    
    // 使用定时器TIM1来做PWM输出
    // 配置TIM1为PWM模式
    // 配置TIM1的预分频器和自动重装载寄存器
    // 配置TIM1的通道1为PWM模式
    // 使能TIM1的通道1输出
    PWM_TimeBaseInitTypeDef PWM_TimeBaseInitType;
	PWM_OCInitTypeDef OutInit;
	/* 时钟选择 */
	PWM_TimeBaseInitType.PWM_ClockSource = PWM_ClockSource_SYSCLK;
	/* 中央计数模式 -- 不开启 */
	PWM_TimeBaseInitType.PWM_CenterAlignedMode = PWM_CenterAlignedMode_Disable;
	/* 计数器计数模式，设置为向上计数 */
	PWM_TimeBaseInitType.PWM_Direction = PWM_Direction_Up;
	/* 驱动CNT计数器的时钟 = Fcksys/(psc+1)*/
    // PSC = 47, 计数频率 = 48MHz / (47+1) = 1MHz, PWM频率 = 1MHz / 1000 = 1kHz
	PWM_TimeBaseInitType.PWM_Prescaler = SERVO_TIMER_PSC;
	/* 周期匹配寄存器,累计MR0+1个频率后产生一个更新或者中断 */
	// PWM_TimeBaseInitType.PWM_AutoReloadValue = 1000;//1ms 触发一次，也就是1kHz
	PWM_TimeBaseInitType.PWM_AutoReloadValue = SERVO_TIMER_ARR;
	/* 初始化TIM1*/
	PWM_TimeBaseInit(TIM1,&PWM_TimeBaseInitType);
	/* 配置为PWM输出通道为1通道*/
	OutInit.PWM_Channel = PWM_Channel_3;
	/* 配置为PWM输出模式 */	
	OutInit.PWM_OCMode = TIM_OCMode_PWM1;
	/* 配置输出和互补输出 */
	OutInit.PWM_OCNOutput = PWM_OCNOutput_Enable;		
	OutInit.PWM_OCOutput = PWM_OCOutput_Disable;  
	/* 设置PWM空闲时候的输出电平状态 */	
	OutInit.PWM_OCIdleState = PWM_OCIdleState_Low;
	OutInit.PWM_OCNIdleState = PWM_OCNIdleState_High;
	/* 配置PWM输出的占空比 P = (PWM_OCValue+1) / (PWM_AutoReloadValue+1)*/	
	OutInit.PWM_OCValue = 1500; // 初始占空比设为 1500us（90°的位置）
    /* 配置PWM比较输出极性*/	
	OutInit.PWM_OCPolarity = PWM_OCPolarity_High;
    OutInit.PWM_OCNPolarity = PWM_OCNPolarity_Low;
	/* 初始化PWM输出 */
	PWM_OCInit(TIM1, &OutInit);
	/* 使能PWM */
	PWM_Cmd(TIM1,ENABLE);
}

/* ---------- 设置 PWM 脉宽（以 timer tick 单位，这里是微秒） ---------- */
inline void Set_Servo_Pulse_us(u16 pulse)
{
    /* 确保脉宽在允许范围内 */
    if (pulse < SERVO_MIN_PULSE) pulse = SERVO_MIN_PULSE;
    if (pulse > SERVO_MAX_PULSE) pulse = SERVO_MAX_PULSE;

    /* 将 OCR1 设为 pulse （计数以 1us 为单位） */
    TIM1->OCR3 = pulse;
	// 等价于 PWM_SetOCxValue(TIM1, PWM_Channel_3, angle);
}

void servo_set_angle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    /* 线性映射 */
    u32 pulse_range = (u32)SERVO_MAX_PULSE - (u32)SERVO_MIN_PULSE; // 2000
    u32 pulse = (u32)SERVO_MIN_PULSE + ( (u32)angle * pulse_range ) / 180u;
    Set_Servo_Pulse_us((u16)pulse);
}