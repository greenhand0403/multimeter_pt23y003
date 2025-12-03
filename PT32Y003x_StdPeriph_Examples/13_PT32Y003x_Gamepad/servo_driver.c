// servo_driver.c
#include "servo_driver.h"

// 内部函数：将角度转换为TIM1的OCR值
// uint16_t angle_to_ocr(uint8_t angle)
// {
//     // 系统时钟假设为48MHz
//     // Prescaler = 47 -> 计数频率 = 48MHz / 48 = 1MHz (1us per tick)
//     // 周期 20ms = 20000 us -> ARR = 19999
//     // 0.5ms = 500 us, 2.5ms = 2500 us
//     // 角度0° -> 500, 180° -> 2500
//     if (angle > 180) angle = 180;
//     return 500 + (2000 * angle) / 180; // 线性映射
// }

void servo_init(void)
{
    // TODO: PB5 SG90舵机驱动需要自己写，20ms周期，0.5ms~2.5ms高电平对应着舵机角度0°~180°
    // 用TIM2基本定时器来做PWM输出
    // GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_4, AFIO_AF_1,ENABLE);//CH3N

    // // 使用定时器TIM1来做PWM输出
    // // 配置TIM1为PWM模式
    // // 配置TIM1的预分频器和自动重装载寄存器
    // // 配置TIM1的通道1为PWM模式
    // // 使能TIM1的通道1输出
    // PWM_TimeBaseInitTypeDef PWM_TimeBaseInitType;
	// PWM_OCInitTypeDef OutInit;

	// /* 时钟选择 */
	// PWM_TimeBaseInitType.PWM_ClockSource = PWM_ClockSource_SYSCLK;
	// /* 中央计数模式 -- 不开启 */
	// PWM_TimeBaseInitType.PWM_CenterAlignedMode = PWM_CenterAlignedMode_Disable;
	// /* 计数器计数模式，设置为向上计数 */
	// PWM_TimeBaseInitType.PWM_Direction = PWM_Direction_Up;
	// /* 周期匹配寄存器,累计MR0+1个频率后产生一个更新或者中断 */
	// PWM_TimeBaseInitType.PWM_AutoReloadValue = 20000 - 1; // 20ms @ 1MHz
	// /* 驱动CNT计数器的时钟 = Fcksys/(psc+1)*/
    // // PSC = 47, 计数频率 = 48MHz / (47+1) = 1MHz, PWM频率 = 1MHz / 1000 = 1kHz
	// PWM_TimeBaseInitType.PWM_Prescaler = 47;
	// /* 初始化TIM1*/
	// PWM_TimeBaseInit(TIM1,&PWM_TimeBaseInitType);
	// /* 配置为PWM输出通道为1通道*/
	// OutInit.PWM_Channel = PWM_Channel_1;
	// /* 配置为PWM输出模式 */	
	// OutInit.PWM_OCMode = TIM_OCMode_PWM1;
	// /* 配置输出和互补输出 */
	// OutInit.PWM_OCNOutput = PWM_OCNOutput_Enable;		
	// OutInit.PWM_OCOutput = PWM_OCOutput_Disable;  
	// /* 设置PWM空闲时候的输出电平状态 */	
	// OutInit.PWM_OCIdleState = PWM_OCIdleState_Low;
	// OutInit.PWM_OCNIdleState = PWM_OCNIdleState_Low;
	// /* 配置PWM输出的占空比 P = (PWM_OCValue+1) / (PWM_AutoReloadValue+1)*/	
	// // OutInit.PWM_OCValue = angle_to_ocr(90); // 初始90度
    // /* 配置PWM比较输出极性*/	
	// OutInit.PWM_OCPolarity = PWM_OCPolarity_High;
    // OutInit.PWM_OCNPolarity = PWM_OCNPolarity_High;
	// /* 初始化PWM输出 */
	// PWM_OCInit(TIM1, &OutInit);
	// /* 使能PWM */
	// PWM_Cmd(TIM1,ENABLE);
}

void servo_set_angle(uint8_t angle)
{
    // PWM_SetOCxValue(TIM1, PWM_Channel_1, angle_to_ocr(angle));
}