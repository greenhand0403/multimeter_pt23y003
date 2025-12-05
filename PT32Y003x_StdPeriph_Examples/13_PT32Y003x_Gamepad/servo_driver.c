// servo_driver.c
#include "servo_driver.h"

// 内部函数：将角度转换为TIM1的OCR值
uint16_t angle_to_ocr(uint8_t angle)
{
    // 系统时钟假设为48MHz
    // Prescaler = 47 -> 计数频率 = 48MHz / 48 = 1MHz (1us per tick)
    // 周期 20ms = 20000 us -> ARR = 19999
    // 0.5ms = 500 us, 2.5ms = 2500 us
    // 角度0° -> 500, 180° -> 2500
    if (angle > 180) angle = 180;
    return 500 + (2000 * angle) / 180; // 线性映射
}

void servo_init(void)
{
    // TODO: PB5 SG90舵机驱动需要自己写，20ms周期，0.5ms~2.5ms高电平对应着舵机角度0°~180°
    // 用TIM2基本定时器来做PWM输出，需要配置GPIO PB5输出
	GPIO_InitTypeDef GPIO_InitStructure;				//定义一个GPIO_InitTypeDef类型的结构体
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;			//选择要控制的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;		//设置引脚模式为通用推挽输出
	GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;	//无偏置
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//调用库函数，初始化GPIO

	NVIC_InitTypeDef NVIC_InitStruct;								//定义一个NVIC_InitTypeDef类型的结构体
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;//定义一个NVIC_InitTypeDef类型的结构体
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 480-1;//48M/480=0.1MHZ
	TIM_TimeBaseInitStruct.TIM_AutoReload = 10;	//0.1MHZ*10=1MHZ
	TIM_TimeBaseInitStruct.TIM_Direction = TIM_Direction_Up;					 //向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);									 //初始化TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_ARI,ENABLE);									//定时中断初始化
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;								//定时中断源设置
	NVIC_InitStruct.NVIC_IRQChannelPriority=0x00;							//中断优先级设置
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;								//使能NVIC控制器
	NVIC_Init(&NVIC_InitStruct);															//初始化NVIC	
	
	TIM_Cmd(TIM2, ENABLE);																		//开启TIM2
}

void servo_set_angle(uint8_t angle)
{
    // PWM_SetOCxValue(TIM1, PWM_Channel_1, angle_to_ocr(angle));
	soft_pwm_set_angle = angle_to_ocr(angle);
}