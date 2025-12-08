// pwm_driver.c
#include "pwm_driver.h"
volatile uint16_t pb5_high_tick;

void pwm_init(void)
{
    // 用TIM2基本定时器来做PWM输出，需要配置GPIO PB5输出
	GPIO_InitTypeDef GPIO_InitStructure;				//定义一个GPIO_InitTypeDef类型的结构体
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;			//选择要控制的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;		//设置引脚模式为通用推挽输出
	GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;	//无偏置
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//调用库函数，初始化GPIO

	GPIO_ResetBits(GPIOB, GPIO_Pin_5);

	NVIC_InitTypeDef NVIC_InitStruct;								//定义一个NVIC_InitTypeDef类型的结构体
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;//定义一个NVIC_InitTypeDef类型的结构体
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 48-1;//48M/48=1MHZ 1us分频
	TIM_TimeBaseInitStruct.TIM_AutoReload = 10-1;// 0.01ms 10us 触发中断计数一次
	TIM_TimeBaseInitStruct.TIM_Direction = TIM_Direction_Up;					 //向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);									 //初始化TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_ARI,ENABLE);									//定时中断初始化

	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;								//定时中断源设置
	NVIC_InitStruct.NVIC_IRQChannelPriority=0x00;							//中断优先级设置
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;								//使能NVIC控制器
	NVIC_Init(&NVIC_InitStruct);															//初始化NVIC	
	
	TIM_Cmd(TIM2, ENABLE);																		//开启TIM2
}

void pwm_set_duty(uint16_t duty)
{
	if (duty > 255) duty = 255;
	// 将0~255换算成0~100的整数
    pb5_high_tick = duty * 100 / 255;
}