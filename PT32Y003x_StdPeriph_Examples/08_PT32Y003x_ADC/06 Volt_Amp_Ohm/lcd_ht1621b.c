#include "lcd_ht1621b.h"

// 建议新增：分别处理 MSB-first 与 LSB-first
static void HT1621_WriteBits_MSB(uint8_t data, uint8_t cnt)
{
    while (cnt--) {
        if (data & 0x80) LCD_DATA_HIGH(); else LCD_DATA_LOW();
        LCD_WR_LOW(); delay_us(2);
        LCD_WR_HIGH(); delay_us(2);
        data <<= 1;
    }
}

// 写 4bit 数据：按 D0..D3（LSB-first）
static void HT1621_Write4_LSB(uint8_t d4) // 只用低 4 位
{
    for (uint8_t i = 0; i < 4; i++) {
        if (d4 & 0x01) LCD_DATA_HIGH(); else LCD_DATA_LOW();
        LCD_WR_LOW(); delay_us(2);
        LCD_WR_HIGH(); delay_us(2);
        d4 >>= 1; // 低位先行
    }
}

// 发送命令：100 + C7..C0 + X
static void HT1621_SendCommand(uint8_t cmd)
{
    LCD_CS_LOW();
    HT1621_WriteBits_MSB(0x80, 3);       // 100
    HT1621_WriteBits_MSB(cmd, 8);        // C7..C0
    HT1621_Write4_LSB(0x0);              // 1 个 don't care（打 1 次 WR 上升沿）
    LCD_CS_HIGH();
}

// 连续写数据：101 + A5..A0 + (D0..D3)*N
void HT1621_WriteData(uint8_t addr, const uint8_t *data, uint8_t lenBytes)
{
    LCD_CS_LOW();
    HT1621_WriteBits_MSB(0xA0, 3);       // 101
    HT1621_WriteBits_MSB(addr << 2, 6);  // A5..A0 (MSB-first)

    // 每个字节拆成两次 4bit，且每 4bit 用 LSB-first
    for (uint8_t i = 0; i < lenBytes; i++) {
        uint8_t b = data[i];
        HT1621_Write4_LSB(b & 0x0F);     // 低 4 位 -> 地址 addr
        HT1621_Write4_LSB(b >> 4);       // 高 4 位 -> 地址 addr+1
    }
    LCD_CS_HIGH();
}

// 推荐的初始化顺序与命令值
void HT1621_Init(void)
{
    // 引脚初始化略
    HT1621_SendCommand(0x01); // SYS EN
    HT1621_SendCommand(0x18); // RC 256K
    HT1621_SendCommand(0x05); // WDT DIS（可选）
    HT1621_SendCommand(0x29); // BIAS=1/3, COM=4
    HT1621_SendCommand(0x03); // LCD ON
    HT1621_Clear();
}

void HT1621_Clear(void)
{
    uint8_t zero[16] = {0};              // 32 个 4bit
    HT1621_WriteData(0x00, zero, 16);
}
