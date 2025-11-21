#include "PT32Y003x.h"
#include "PT32Y003x_gpio.h"
#include "PT32Y003x_uart.h"
#include "PT32Y003x_nvic.h"
#include "PT32Y003x_i2c.h"
#include "PT32Y003x_tim.h"
#include <stdio.h>
#include <PT32Y003x_rcc.h>
#include <stdarg.h>
#include "delay.h"

#pragma region 宏和全局变量
// ===== 按键定义 =====
#define KEY_UP      GPIO_ReadDataBit(GPIOA, GPIO_Pin_1)   // PA1
#define KEY_LEFT    GPIO_ReadDataBit(GPIOA, GPIO_Pin_2)   // PA2
#define KEY_DOWN    GPIO_ReadDataBit(GPIOC, GPIO_Pin_3)   // PC3
#define KEY_RIGHT   GPIO_ReadDataBit(GPIOC, GPIO_Pin_4)   // PC4
#define KEY_A       GPIO_ReadDataBit(GPIOC, GPIO_Pin_5)   // PC5
#define KEY_B       GPIO_ReadDataBit(GPIOC, GPIO_Pin_6)   // PC6

// 按键有效电平：低电平有效（0 = pressed）
#define KEY_PRESSED 0

// ===== 蓝牙模块配置 =====
#define BLUETOOTH_BAUD 115200
// ===== 调试串口配置 =====
#define DEBUG_BAUD 115200

// ===== 蓝牙消息包格式：一共四字节，中间是数据和校验和 =====
#define PACKET_HEADER   0xAA
#define PACKET_TAIL     0x55

// ===== 全局变量 =====
u8 last_key_state = 0;
u8 current_key_state = 0;

// ===== 蓝牙模块状态 =====
// 给中断服务函数用，标志蓝牙模块是否初始化完成
u8 bluetooth_ready = 0;

// ===== UART接收缓冲区 =====
// 给中断服务函数用，用于存储从蓝牙模块接收的数据
u8 rx_buffer[64];
u16 rx_index = 0;
#pragma endregion

#pragma region 函数声明
// ===== 函数声明 =====

void GPIO_Config(void);
void UART0_Config(void);
void UART1_Config(void);
void I2C0_Config(void);
void Servo_Config(void);
void Timer_Config(void);
void Bluetooth_Init(void);
void SendKeyStatePacket(void);
void SendDebugInfo(void);

u8 CalculateChecksum(u8* data, u8 len);
#pragma endregion

#pragma region 串口格式化输出辅助函数
// 向指定UART发送字符串
void UART_SendString(UART_TypeDef* UARTx, const char *str)
{
    while (*str) {
        UART_SendData(UARTx, *str);
        while (UART_GetFlagStatus(UARTx, UART_FLAG_TXE) == RESET);
        str++;
    }
}
// 核心的格式化输出函数（支持基本格式）
void UART_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    UART_SendString(UART0, buffer);
    UART_SendString(UART1, buffer);
}

// 专门输出到UART0（蓝牙）
void Bluetooth_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    UART_SendString(UART0, buffer);
}

// 专门输出到UART1（调试）
void Debug_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    UART_SendString(UART1, buffer);
}
#pragma endregion

#pragma region 主函数
// ===== 主函数 =====
int main(void)
{
    SysTick_Init();
    
    // 初始化所有外设
    GPIO_Config();
    UART0_Config();
    UART1_Config();
    // 预留 陀螺仪初始化
    // I2C0_Config();
    // 预留 舵机初始化
    // Servo_Config();
    // Timer_Config();
    
    UART_Printf("UART0 UART1 OK\r\n");
    
    // 初始化蓝牙模块
    Bluetooth_Init();
    
    UART_Printf("BT Init OK\r\n");
    
    while (1)
    {
        // 读取当前按键状态
        current_key_state = 0;
        if (KEY_UP == KEY_PRESSED)      current_key_state |= 0x01;
        if (KEY_LEFT == KEY_PRESSED)    current_key_state |= 0x02;
        if (KEY_DOWN == KEY_PRESSED)    current_key_state |= 0x04;
        if (KEY_RIGHT == KEY_PRESSED)   current_key_state |= 0x08;
        if (KEY_A == KEY_PRESSED)       current_key_state |= 0x10;
        if (KEY_B == KEY_PRESSED)       current_key_state |= 0x20;
        
        // 检测按键状态变化
        if (current_key_state != last_key_state)
        {
            SendKeyStatePacket();
            SendDebugInfo();
            last_key_state = current_key_state;
        }
        
        delay_ms(100); // 防抖延时
    }
}

// ===== GPIO初始化 =====
void GPIO_Config(void)
{
    GPIO_InitTypeDef gpio;
    
    // 配置按键输入（PA1, PA2, PC3, PC4, PC5, PC6）- 上拉输入
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOA, &gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOC, &gpio);
    
    /* 配置 PD5 (TX0) 为 AF 推挽输出，PD6 (RX0) 为输入浮空或上拉 */
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_5, AFIO_AF_0,ENABLE);	//PD5 TX0
    GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_6, AFIO_AF_0,ENABLE);	//PD6 RX0

    /* UART1 remap -> PB1 TX1 */
    GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_1, AFIO_AF_1,ENABLE);	//PB1 TX1
    
    // 配置I2C0引脚（PB4=SCL, PB5=SDA）
    GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_4, AFIO_AF_0, ENABLE); // PB4 = SCL
    GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_5, AFIO_AF_0, ENABLE); // PB5 = SDA
    
    // 配置舵机引脚（PA3）
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OutPP;
    // gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); // 默认低电平
}

// ===== UART0配置（蓝牙通信）=====
void UART0_Config(void)
{
    UART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    
    // NVIC配置
    nvic.NVIC_IRQChannel = UART0_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x01;
    NVIC_Init(&nvic);

    // 使能接收中断
    UART_ITConfig(UART0, UART_IT_RXNEI, ENABLE);
    
    // UART0配置
    uart.UART_BaudRate = BLUETOOTH_BAUD;
    uart.UART_WordLengthAndParity = UART_WordLengthAndParity_8D;
    uart.UART_StopBitLength = UART_StopBitLength_1;
    uart.UART_ParityMode = UART_ParityMode_Odd;
    uart.UART_Receiver = UART_Receiver_Enable;
    uart.UART_LoopbackMode = UART_LoopbackMode_Disable;
    
    UART_Cmd(UART0, ENABLE);
	UART_Init(UART0, &uart);
}

// ===== UART1配置（调试输出）=====
void UART1_Config(void)
{
    UART_InitTypeDef uart;
    
    uart.UART_BaudRate = DEBUG_BAUD;
    uart.UART_WordLengthAndParity = UART_WordLengthAndParity_8D;
    uart.UART_StopBitLength = UART_StopBitLength_1;
    uart.UART_ParityMode = UART_ParityMode_Odd;
    // uart.UART_Receiver = UART_Receiver_Enable;
    uart.UART_LoopbackMode = UART_LoopbackMode_Disable;

    UART_Cmd(UART1, ENABLE);
    UART_Init(UART1, &uart);
}

// ===== I2C0配置（预留MPU6050）=====
void I2C0_Config(void)
{
    I2C_InitTypeDef i2c;
    
    i2c.I2C_Acknowledge = I2C_Acknowledge_Enable;
    i2c.I2C_Broadcast = I2C_Broadcast_Disable;
    i2c.I2C_OwnAddress = 0x00;
    i2c.I2C_Prescaler = 479; // 假设PCLK=48MHz, SCL≈100kHz
    I2C_Init(I2C0, &i2c);
    I2C_Cmd(I2C0, ENABLE);
}

// ===== 舵机配置（PA3）=====
void Servo_Config(void)
{
    // 预留：配置PA3为PWM输出控制SG90
    // 需要使用定时器输出50Hz PWM信号
    // 这里先配置为普通GPIO
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OutPP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
}

// ===== 定时器配置（预留PWM）=====
void Timer_Config(void)
{
    // 预留：用于舵机PWM或其他定时任务

    // 这里可以配置TIMER1或TIMER2
}

// ===== 蓝牙模块初始化 =====
void Bluetooth_Init(void)
{
    // 只发给蓝牙，查询波特率
    Bluetooth_Printf("AT+QT\r\n"); // 查询波特率
    // 只发给 UART1 调试
    Debug_Printf("DEBUG UART1\r\n");
    // 延长1秒测试
    delay_ms(1000);
    UART_Printf("All OK\r\n");
}

// ===== 发送按键状态数据包（标准4字节格式）=====
void SendKeyStatePacket(void)
{
    u8 packet[4]; // 头部 + 数据 + 校验 + 尾部
    
    packet[0] = PACKET_HEADER;      // 包头
    packet[1] = current_key_state;  // 按键状态（低6位）
    packet[2] = CalculateChecksum(&packet[0], 2); // 校验和
    packet[3] = PACKET_TAIL;        // 包尾
    
    // 发送整个数据包
    for (int i = 0; i < 4; i++)
    {
        UART_SendData(UART0, packet[i]);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TXE) == RESET); // 等待发送完成
    }
    
    Debug_Printf("Sent key packet: %02X %02X %02X %02X\r\n", 
           packet[0], packet[1], packet[2], packet[3]);
}

// ===== 计算校验和 =====
u8 CalculateChecksum(u8* data, u8 len)
{
    u8 sum = 0;
    for (u8 i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

// ===== 只发送调试信息到UART1 =====
void SendDebugInfo(void)
{
    Debug_Printf("Key State: 0x%02X -> ", current_key_state);
    
    if (current_key_state & 0x01) Debug_Printf("UP ");
    if (current_key_state & 0x02) Debug_Printf("LEFT ");
    if (current_key_state & 0x04) Debug_Printf("DOWN ");
    if (current_key_state & 0x08) Debug_Printf("RIGHT ");
    if (current_key_state & 0x10) Debug_Printf("A ");
    if (current_key_state & 0x20) Debug_Printf("B ");
    Debug_Printf("\r\n");
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