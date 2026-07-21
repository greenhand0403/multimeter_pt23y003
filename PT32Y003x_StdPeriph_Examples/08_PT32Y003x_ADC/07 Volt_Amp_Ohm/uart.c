#include "uart.h"

#include <PT32Y003x_uart.h>
#include <PT32Y003x_gpio.h>

#if ENABLE_LOG

char log_buffer[LOG_BUFFER_SIZE];
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
        GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0,ENABLE);	    //PD5 TX0
        // GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0,ENABLE);	//PD6 RX0 会影响PA1！详情就见文档？
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
    // 不使用 RX引脚 不接收串口数据
	UART_InitStruct.UART_Receiver=UART_Receiver_Disable;
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
void UART_SendString(const char* str)
{
    while (*str)
    {
        UART_SendData(LOG_UART, *str++);
        while (UART_GetFlagStatus(LOG_UART, UART_FLAG_TXE) == RESET);
    }
}
#endif
