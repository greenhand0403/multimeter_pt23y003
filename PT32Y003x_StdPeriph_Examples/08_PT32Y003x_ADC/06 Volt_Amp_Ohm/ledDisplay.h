#ifndef LED_DISPLAY_H
#define LED_DISPLAY_H

#include <stdint.h>
#include "PT32Y003x.h"

// 显示缓冲区：最多5位，前4位为数字，第5位为单位/符号
extern volatile uint8_t display_buffer[5];  // 每位的段码

void LEDDisplay_Init(void);
void LEDDisplay_UpdateColumn(void);
void LEDDisplay_SetDigit(int index, uint8_t seg_code);
void LEDDisplay_Clear(void);
void LEDDisplay_SetALL(void);
#endif
