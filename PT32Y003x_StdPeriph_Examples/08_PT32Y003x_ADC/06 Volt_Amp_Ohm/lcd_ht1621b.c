#include "lcd_ht1621b.h"
#define DelayT delay_us(1000)
// 建议新增：分别处理 MSB-first 与 LSB-first
static void HT1621_WriteBits_MSB(uint8_t data, uint8_t cnt)
{
    while (cnt--) {
        // WR 低电平使能 开始写
        LCD_WR_LOW(); DelayT;
        if (data & 0x80) LCD_DATA_HIGH(); else LCD_DATA_LOW();
        DelayT;
        // WR 高电平使能 结束写
        LCD_WR_HIGH(); DelayT;
        data <<= 1;
    }
}

// 写 4bit 数据：按 D0..D3（LSB-first）
static void HT1621_Write4_LSB(uint8_t d4) // 只用低 4 位
{
    for (uint8_t i = 0; i < 4; i++) {
        LCD_WR_LOW(); DelayT;
        if (d4 & 0x01) LCD_DATA_HIGH(); else LCD_DATA_LOW();
        DelayT;
        LCD_WR_HIGH(); DelayT;
        d4 >>= 1; // 低位先行
    }
}
// 仅打一位 don't care（无所谓 0/1，只需 1 次 WR 上升沿）
static inline void HT1621_Write1bit(uint8_t bit)
{
    LCD_WR_LOW();  DelayT;
    if (bit) LCD_DATA_HIGH(); else LCD_DATA_LOW();
    DelayT;
    LCD_WR_HIGH(); DelayT;
}

/// @brief 发送命令
/// @param cmd 命令
/// 发送命令：100 + C7..C0 + X
void HT1621_SendCommand(uint8_t cmd)
{
    // CS 低电平使能
    LCD_CS_LOW();
    HT1621_WriteBits_MSB(0x80, 3);       // 100
    HT1621_WriteBits_MSB(cmd, 8);        // C7..C0
    // HT1621_Write4_LSB(0x0);              // 1 个 don't care（打 1 次 WR 上升沿）
    HT1621_Write1bit(0);
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

/// @brief 清楚所有段
/// @param  
void HT1621_Clear(void) {
    uint8_t z[16];
    for (int i = 0; i < 16; ++i) z[i] = 0x00;   // 覆盖 0x00..0x1F 共 32 个 4bit
    HT1621_WriteData(0x00, z, 16);
}
// 推荐的初始化顺序与命令值
void HT1621_Init(void)
{
    LCD_CS_HIGH();
    LCD_WR_HIGH();
    LCD_DATA_HIGH();

    // 引脚初始化略
    HT1621_SendCommand(0x01); // SYS EN
    HT1621_SendCommand(0x18); // RC 256K
    HT1621_SendCommand(0x29); // BIAS=1/3, COM=4
    // HT1621_SendCommand(0x05); // WDT DIS（可选）
    HT1621_SendCommand(0x03); // LCD ON
    HT1621_Clear();
}

// ===== 显示数字（0~9）的 7 段映射：{L_nibble, R_nibble} =====
// 段位命名 ABCDEFG，顺时针从顶横开始，G 为中横。
// L: A(0x1) F(0x2) E(0x4) D(0x8)
// R: B(0x1) G(0x2) C(0x4) [R的0x8可留作DP]
static const uint8_t kDigitMap_7seg[10][2] = {
    /*0*/ {0x0F, 0x05},  // A F E D  +  B C
    /*1*/ {0x00, 0x05},  //           +  B C
    /*2*/ {0x0D, 0x03},  // A   E D  +  B G
    /*3*/ {0x09, 0x07},  // A     D  +  B G C
    /*4*/ {0x02, 0x07},  //   F      +  B G C
    /*5*/ {0x0B, 0x06},  // A F   D  +    G C
    /*6*/ {0x0F, 0x06},  // A F E D  +    G C
    /*7*/ {0x01, 0x05},  // A         +  B   C
    /*8*/ {0x0F, 0x07},  // A F E D  +  B G C
    /*9*/ {0x0B, 0x07},  // A F   D  +  B G C
};
    
// 每一位的“左地址”（右地址=左地址+1）
static const uint8_t kDigitAddrL[4] = {
    ADDR_FIRST_L,  ADDR_SECOND_L,  ADDR_THIRD_L,  ADDR_FOURTH_L
};
    
// 写入“某一位”的 L/R 两个 4bit（一次发 1 字节，低4写L，高4写R）
static inline void LCD_WriteDigitPair(uint8_t addrL, uint8_t nibL, uint8_t nibR)
{
    uint8_t b = (uint8_t)((nibR << 4) | (nibL & 0x0F));
    HT1621_WriteData(addrL, &b, 1);
}
/// @brief 清 4 位（仅数码段，不动其它未用地址）
/// @param 无
/// @return 无
void LCD_Clear4Digits(void)
{
    uint8_t z = 0x00;
    for (int i = 0; i < 4; ++i) {
        HT1621_WriteData(kDigitAddrL[i], &z, 1);
    }
}
/// @brief 显示数字
/// @param pos 位置
/// @param val 值
/// @param dp 是否显示小数点
/// pos: 0..3（从左到右），val: 0..9，dp=true 则在该位右半字节加小数点(0x8)
void LCD_ShowDigit(uint8_t pos, uint8_t val, bool dp)
{
    if (pos > 3 || val > 9) return;
    uint8_t nibL = kDigitMap_7seg[val][0];
    uint8_t nibR = kDigitMap_7seg[val][1];
    if (dp) nibR |= 0x8;
    LCD_WriteDigitPair(kDigitAddrL[pos], nibL, nibR);
}
    
/// @brief 显示简单整数 0000~9999（不加图标/小数点）
/// @param value 值
void LCD_ShowNumber4(uint16_t value)
{
    if (value > 9999) value = 9999;
    uint8_t d0 = (uint8_t)((value / 1000) % 10);
    uint8_t d1 = (uint8_t)((value / 100)  % 10);
    uint8_t d2 = (uint8_t)((value / 10)   % 10);
    uint8_t d3 = (uint8_t)( value          % 10);

    LCD_ShowDigit(0, d0, false);
    LCD_ShowDigit(1, d1, false);
    LCD_ShowDigit(2, d2, false);
    LCD_ShowDigit(3, d3, false);
}

static bool last_minus = false;
static bool last_ovf   = false;
// === 电压表专用 ===
// scaled_2dp = |V| * 100（四舍五入），范围 0..1200（外部已经钳位到 12.00）
// - 第 dot_pos 位（从左数起1开始）：点亮小数点
void LCD_Show_digits(uint16_t scaled_2dp, uint8_t dot_pos)
{
    if (dot_pos==4)
    {
        // 显示欧姆表未接入的状态 - - - -
        uint8_t tmp = (uint8_t)(0x2 << 4);
        for (uint8_t addr = ADDR_FIRST_L; addr < ADDR_FIRST_L+8; addr+=2)
        {
            HT1621_WriteData(addr, &tmp, 1);
        }
        return;
    }
    

    if (scaled_2dp > 9999) scaled_2dp = 9999;

    uint8_t d0 = (uint8_t)((scaled_2dp / 1000) % 10);
    uint8_t d1 = (uint8_t)((scaled_2dp / 100)  % 10);
    uint8_t d2 = (uint8_t)((scaled_2dp / 10)   % 10);
    uint8_t d3 = (uint8_t)( scaled_2dp         % 10);

    // 位0：千位。为美观，千位=0时可留空（你要保留前导零就改成 LCD_ShowDigit(0, d0, false)）
    // if (d0 == 0) {
        // uint8_t z = 0x00;
        // HT1621_WriteData(ADDR_FIRST_L, &z, 1);
    // } else {
        LCD_ShowDigit(0, d0, dot_pos==1);
    // }

    // 位1：百位 + V 符号 + 中间小数点 + 负号/溢出标志
    // 先拿到“数字 0~9”的段
    // uint8_t nibL = kDigitMap_7seg[d1][0];
    // uint8_t nibR = kDigitMap_7seg[d1][1];
    // nibR |= 0x8;            // 第二位的小数点 DP（“中间那个小数点”）
    // LCD_WriteDigitPair(ADDR_SECOND_L, nibL, nibR);
    LCD_ShowDigit(1, d1, dot_pos==2);
    // 位2、位3：十位、个位
    LCD_ShowDigit(2, d2, dot_pos==3);

    LCD_ShowDigit(3, d3, false);
}

void LCD_ShowIcon(uint8_t icon1, uint8_t icon2)
{
    HT1621_WriteData(ADDR_AMPMA_OVERF_ALR_NEG, &icon1, 1);
    HT1621_WriteData(ADDR_BAT100_BAT75, &icon2, 1);
}

// 遍历 SEG9~SEG20（0x09~0x14），每次只点亮一个段，停 5 秒
void LCD_SegWalkTest(void)
{
    while (1) {
        for (uint8_t addr = 0x09; addr <= 0x14; ++addr) {
            for (uint8_t bit = 0x01; bit <= 0x08; bit <<= 1) {
                HT1621_Clear();                 // 先清屏，避免残影

                // 计算“成对写”的起始地址（奇数=自身；偶数=addr-1）
                uint8_t base = (addr & 1) ? addr : (uint8_t)(addr - 1);

                // 只点亮目标半字节：目标是 L → low=bit；目标是 R → high=bit
                uint8_t low  = (addr == base) ? bit : 0x00;
                uint8_t high = (addr == base) ? 0x00 : bit;

                uint8_t one_byte = (uint8_t)((high << 4) | (low & 0x0F));
                HT1621_WriteData(base, &one_byte, 1);

                delay_ms(1000);                 // 每段观察 1 秒
            }
        }
    }
}
void LCD_AllOn(void)
{
    // 覆盖 0x00..0x1F（32×4bit）→ 全段点亮
    uint8_t ff[16];
    for (int i = 0; i < 16; ++i) ff[i] = 0xFF;
    HT1621_WriteData(0x00, ff, 16);
}