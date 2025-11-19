#include "PT32Y003x.h"
#include <PT32Y003x_gpio.h>
#include <PT32Y003x_uart.h>
#include <PT32Y003x_nvic.h>

#pragma region 串口配置
void UART_GPIO_Config(void)
{
	/* 配置UART管脚的复用功能 */
	GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0,ENABLE);	//PD5 TX0
	GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0,ENABLE);	//PD6 RX0
}

void UART_Mode_Config(void)
{
	UART_InitTypeDef  UART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;							//定义一个NVIC_InitTypeDef类型的结构体
	
	/*NVIC配置*/
	NVIC_InitStruct.NVIC_IRQChannel = UART0_IRQn;				//设置中断向量号
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				//设置是否使能中断
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;				//设置中断优先级
	NVIC_Init(&NVIC_InitStruct);	
	UART_ITConfig(UART0,UART_IT_RXNEI,ENABLE);
	
	/*初始化UART0*/
	UART_InitStruct.UART_BaudRate = 9600;
	UART_InitStruct.UART_WordLengthAndParity = UART_WordLengthAndParity_8D;
	UART_InitStruct.UART_StopBitLength = UART_StopBitLength_1;
	UART_InitStruct.UART_ParityMode = UART_ParityMode_Even;
	UART_InitStruct.UART_Receiver=UART_Receiver_Enable;
	UART_InitStruct.UART_LoopbackMode=UART_LoopbackMode_Disable;
	UART_Init(UART0, &UART_InitStruct);

	/*开启UART0的收发功能*/
	UART_Cmd(UART0, ENABLE);
}

void UART_Driver(void)
{
	UART_GPIO_Config();
	UART_Mode_Config();
}
#pragma endregion

#pragma region LED指示
void GPIO_Driver(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	//定义一个GPIO_InitTypeDef类型的结构体
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//选择要控制的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;		//设置引脚模式为通用推挽输出
	GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;	//无偏置
	GPIO_Init(GPIOD, &GPIO_InitStructure);	//调用库函数，初始化GPIO
	GPIO_SetBits(GPIOD, GPIO_Pin_4);			//调用库函数，设置GPIO状态
	// 低电平有效 点亮LED
}
#pragma endregion

#pragma region 按键配置
void User_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_In;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Pull=GPIO_Pull_Up;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_3, AFIO_AF_None,DISABLE);
	GPIO_SetBits(GPIOC,GPIO_Pin_3);
}
#pragma endregion

extern u16 data_rx[20];
volatile u8 rx_cnt = 0;
u8 tx_cnt = 0;
int main (void)
{
	// 按键配置
	User_Key_Config();

	u8 i=0;
	UART_Driver();
	printf("hello PT32Y003F4P6B Gamepad \r\n");

	while(1)
  	{
		if (rx_cnt != tx_cnt) {
			UART_SendData(UART0, data_rx[tx_cnt++]);
			if (tx_cnt == rx_cnt) {
				tx_cnt = 0;
				rx_cnt = 0;  // 此处仍需保护，但风险降低
			}
		}
	}
}
	
 
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
	printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
	/* Infinite loop */
	while (1)
	{
	}
}
#endif
