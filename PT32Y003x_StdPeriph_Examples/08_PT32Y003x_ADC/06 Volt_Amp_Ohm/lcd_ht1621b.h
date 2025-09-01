#ifndef __LCD_HT1621B_H__
#define __LCD_HT1621B_H__

#include <stdint.h>
#include <stdbool.h>
#include "PT32Y003x.h"
#include "PT32Y003x_gpio.h"
#include "delay.h"

#define ADDR_BAT50_BAT25_HZ_MOHM 0x14
#define ADDR_BAT100_BAT75        0x13
#define ADDR_AMPA_VOLT_OHM_KOHM  0x12
#define ADDR_AMPMA_OVERF_ALR_NEG 0x11
#define ADDR_FOURTH_R            0x10
#define ADDR_FOURTH_L            0x0F
#define ADDR_THIRD_R             0x0E
#define ADDR_THIRD_L             0X0D
#define ADDR_SECOND_R            0x0C
#define ADDR_SECOND_L            0x0B
#define ADDR_FIRST_R             0x0A
#define ADDR_FIRST_L             0x09

#define ICON_NEG                 0x01
#define ICON_OVERF				 0x04

#define ICON_VOLT		         0x04
#define ICON_OHM		         0x02
#define ICON_AMP_A		         0x08
#define ICON_AMP_MA              0x08
#define ICON_OHM_KO		         0x01
#define ICON_OHM_MO		         0x01

#define ICON_BAT_25              0x04
#define ICON_BAT_50              0x08
#define ICON_BAT_75              0x04
#define ICON_BAT_100             0x08
#define ICON_BAT_BROAD           0x02

// CS_ PB5 WR_ PB4 DATA PC3
#define LCD_CS_HIGH()     GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define LCD_CS_LOW()      GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define LCD_WR_HIGH()     GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define LCD_WR_LOW()      GPIO_ResetBits(GPIOB, GPIO_Pin_4)

#define LCD_DATA_HIGH()   GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define LCD_DATA_LOW()    GPIO_ResetBits(GPIOC, GPIO_Pin_3)

void HT1621_Init(void);
// void HT1621_WriteCmd(uint8_t cmd);
void HT1621_WriteData(uint8_t addr, const uint8_t *data, uint8_t len);
void HT1621_Clear(void);
void HT1621_SendCommand(uint8_t cmd);
void LCD_ShowDigit(uint8_t pos, uint8_t val, bool dp);
void LCD_ShowNumber4(uint16_t value);
void LCD_Clear4Digits(void);

void LCD_Show_digits(uint16_t scaled_2dp, uint8_t dot_pos);
void LCD_ShowIcon(uint8_t icon1, uint8_t icon2);
void LCD_SegWalkTest(void);
void LCD_AllOn(void);
#endif
