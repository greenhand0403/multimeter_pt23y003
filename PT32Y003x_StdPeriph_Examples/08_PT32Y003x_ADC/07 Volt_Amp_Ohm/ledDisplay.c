#include "ledDisplay.h"
#include <PT32Y003x_gpio.h>
// 显示1234
// LEDDisplay_SetDigit(0, 0x06); // 显示1
// LEDDisplay_SetDigit(1, 0x5B); // 显示2
// LEDDisplay_SetDigit(2, 0x4F); // 显示3
// LEDDisplay_SetDigit(3, 0x66); // 显示4
// LEDDisplay_SetDigit(4, 0x01); // 显示"V"（例如自定义）
// 严谨一点可以写成只读的常量，写成变量传参给函数编译时不会报格式转换的警告
// 段码定义（5位共阳，1点亮）
GPIO_TypeDef * COM_PORT_ARRAY[] = {GPIOD, GPIOB, GPIOB, GPIOB, GPIOC};
uint16_t COM_PIN_ARRAY[] = {GPIO_Pin_4, GPIO_Pin_1, GPIO_Pin_5, GPIO_Pin_4, GPIO_Pin_7};
GPIO_TypeDef * SEG_PORT_ARRAY[] = {GPIOC, GPIOC, GPIOC, GPIOC, GPIOD, GPIOD, GPIOD};
uint16_t SEG_PIN_ARRAY[] = {GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3};

// 当前正在刷新的位
static volatile int current_col = 0;
volatile uint8_t display_buffer[5] = {0};  // 每位7段的段码

void LEDDisplay_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    for (int i = 0; i < 5; i++) {
        GPIO_InitStructure.GPIO_Pin = COM_PIN_ARRAY[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
        GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;
        GPIO_Init(COM_PORT_ARRAY[i], &GPIO_InitStructure);
        GPIO_SetBits(COM_PORT_ARRAY[i], COM_PIN_ARRAY[i]);
    }

    for (int i = 0; i < 7; i++) {
        GPIO_InitStructure.GPIO_Pin = SEG_PIN_ARRAY[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
        GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;
        GPIO_Init(SEG_PORT_ARRAY[i], &GPIO_InitStructure);
        GPIO_ResetBits(SEG_PORT_ARRAY[i], SEG_PIN_ARRAY[i]);
    }
	// 禁用 SWD 接口，使SW调试复用的 PD1,PC7 可自由使用
	GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_7, AFIO_AF_None, DISABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
	GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	GPIO_DigitalRemapConfig(AFIOD, GPIO_Pin_1, AFIO_AF_None, DISABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
	GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_1);
}

void LEDDisplay_UpdateColumn(void)
{
    // 关闭当前列
    GPIO_SetBits(COM_PORT_ARRAY[current_col], COM_PIN_ARRAY[current_col]);

    // 清除所有段
    for (int i = 0; i < 7; i++) {
        GPIO_ResetBits(SEG_PORT_ARRAY[i], SEG_PIN_ARRAY[i]);
    }

    // 切换到下一列
    current_col = (current_col + 1) % 5;
    uint8_t seg_code = display_buffer[current_col];

    // 设置新段码（低位代表SEG0）
    for (int i = 0; i < 7; i++) {
        if (seg_code & (1 << i)) {
            GPIO_SetBits(SEG_PORT_ARRAY[i], SEG_PIN_ARRAY[i]); // 点亮段
        }
    }

    // 开启新列
    GPIO_ResetBits(COM_PORT_ARRAY[current_col], COM_PIN_ARRAY[current_col]);
}

void LEDDisplay_SetDigit(int index, uint8_t seg_code)
{
    if (index >= 0 && index < 5) {
        display_buffer[index] = seg_code;
    }
}

void LEDDisplay_Clear(void)
{
    for (int i = 0; i < 5; i++) {
        display_buffer[i] = 0;
    }
}
void Display_SetAll(void)
{
	// 点亮所有段
	GPIO_InitTypeDef GPIO_InitStructure;
	for (int i = 0; i < 5; i++) {
		GPIO_InitStructure.GPIO_Pin = COM_PIN_ARRAY[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
		GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;
		GPIO_Init(COM_PORT_ARRAY[i], &GPIO_InitStructure);
		GPIO_ResetBits(COM_PORT_ARRAY[i], COM_PIN_ARRAY[i]);
	}

	for (int i = 0; i < 7; i++) {
		GPIO_InitStructure.GPIO_Pin = SEG_PIN_ARRAY[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
		GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull;
		GPIO_Init(SEG_PORT_ARRAY[i], &GPIO_InitStructure);
		GPIO_SetBits(SEG_PORT_ARRAY[i], SEG_PIN_ARRAY[i]);
	}
}