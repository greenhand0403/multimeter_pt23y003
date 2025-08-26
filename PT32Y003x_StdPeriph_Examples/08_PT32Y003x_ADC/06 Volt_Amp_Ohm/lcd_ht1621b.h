#ifndef __LCD_HT1621B_H__
#define __LCD_HT1621B_H__

#include <stdint.h>
#include <stdbool.h>
#include "PT32Y003x.h"
#include "PT32Y003x_gpio.h"
#include "delay.h"

#define HALF_BAT_ADDR           0x09
#define FULL_BAT_ADDR           0x0A
#define AMP_ADDR                0x0B
#define MILLI_AMP_ADDR          0x0C
#define COLON_ADDR              0x0D
#define FOURTH_8_ADDR           0x0E
#define RIGHT_POINT_ADDR        0x0F
#define THIRD_8_ADDR            0X10
#define MIDDLE_POINT_ADDR       0x11
#define SECOND_8_ADDR           0x12
#define LEFT_POINT_ADDR         0x13
#define FIRST_8_ADDR            0x14

#define DISPLAY_ALARM           0x04
#define DISPLAY_MOhm            0x01
#define DISPLAY_HALF_BAT        0x08
#define DISPLAY_HZ				0x02

// CS PB5 WR PB4 DATA PC3
#define LCD_CS_HIGH()     GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define LCD_CS_LOW()      GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define LCD_WR_HIGH()     GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define LCD_WR_LOW()      GPIO_ResetBits(GPIOB, GPIO_Pin_4)

#define LCD_DATA_HIGH()   GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define LCD_DATA_LOW()    GPIO_ResetBits(GPIOC, GPIO_Pin_3)

void HT1621_Init(void);
void HT1621_WriteCmd(uint8_t cmd);
void HT1621_WriteData(uint8_t addr, const uint8_t *data, uint8_t len);
void HT1621_Clear(void);

#endif
