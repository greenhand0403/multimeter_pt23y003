// gyro_driver.c
#include "gyro_driver.h"
#include "delay.h"
// 返回1=等到了，0=超时
static uint8_t i2c_wait_flag_set(uint32_t flag, uint32_t timeout_ms)
{
    uint32_t t0 = s_ms_ticks; // 你的1ms计数（volatile）
    while (I2C_GetFlagStatus(I2C0, flag) != SET) {
        if ((uint32_t)(s_ms_ticks - t0) >= timeout_ms) {
            return 0;
        }
    }
    return 1;
}

void gyro_init(void)
{
    /* 配置I2C管脚的复用功能 */
	GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_5, AFIO_AF_0,ENABLE);	//PB5 I2C SDA
	GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_4, AFIO_AF_0,ENABLE);	//PB4 I2C SCL

    I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Acknowledge = I2C_Acknowledge_Disable;
	I2C_InitStruct.I2C_Broadcast = I2C_Broadcast_Disable;
	I2C_InitStruct.I2C_OwnAddress = 0x00;
	I2C_InitStruct.I2C_Prescaler = 479;
	I2C_Init(I2C0,&I2C_InitStruct);

    I2C_Cmd(I2C0, ENABLE);
}
static void i2c_recover(void)
{
    I2C_Cmd(I2C0, DISABLE);
    // 可选：延时几百us
    I2C_Cmd(I2C0, ENABLE);
}
void I2C_EE_Read(uint8_t* pBuffer,uint16_t ReadAddr, uint16_t DeviceAddr, uint16_t data_size)
{
	int i;
/******************等待从机ready***************/		
	I2C_GenerateEvent(I2C0,I2C_Event_Start,ENABLE);
	if (!i2c_wait_flag_set(I2C_FLAG_StartOk, 5)) goto fail;
	I2C_SendAddr(I2C0, DeviceAddr);//器件地址，写
	if (!i2c_wait_flag_set(I2C_FLAG_MASGetAckW, 5)) goto fail;
	I2C_SendData(I2C0,ReadAddr);//发送要读的页地址
	if (!i2c_wait_flag_set(I2C_FLAG_MDSGetAck, 5)) goto fail;
	I2C_GenerateEvent(I2C0,I2C_Event_Stop,ENABLE);
/******************接收数据***************/		
	I2C_GenerateEvent(I2C0,I2C_Event_Start,ENABLE);
	if (!i2c_wait_flag_set(I2C_FLAG_StartOk, 5)) goto fail;
	I2C_SendAddr(I2C0, DeviceAddr|0x1);//器件地址，读
	if (!i2c_wait_flag_set(I2C_FLAG_MASGetAckR, 5)) goto fail;
	I2C0->CR=I2C_CR_ACK;
	for(i=0;i<data_size;i++)
	{
		if(i==(data_size-1))
		{
			I2C0->CCR=I2C_CCR_ACK|I2C_CCR_SI;//主机发送NACK
			if (!i2c_wait_flag_set(I2C_FLAG_MDGSendNack, 5)) goto fail;
			*pBuffer=I2C_ReceiveData(I2C0);
			pBuffer++;
			break;
		}
		I2C0->CCR=I2C_CCR_SI;
		if (!i2c_wait_flag_set(I2C_FLAG_MDGSendAck, 5)) goto fail;
		*pBuffer=I2C_ReceiveData(I2C0);
		pBuffer++;
	}
	/******************发送停止位***************/
	I2C_GenerateEvent(I2C0,I2C_Event_Stop,ENABLE);
    return;          // ★ 成功直接返回
    fail:
    i2c_recover();
}
void I2C_EE_Write(uint8_t* pBuffer, unsigned int WriteAddr,uint16_t DeviceAddr, uint16_t data_size)
{
	int i;
/******************等待从机ready***************/		
	I2C_GenerateEvent(I2C0,I2C_Event_Start,DISABLE);
	I2C0->CCR |= I2C_CCR_SI | I2C_CCR_ACK;
	I2C_Cmd(I2C0,DISABLE);
	I2C_Cmd(I2C0,ENABLE);
	I2C_GenerateEvent(I2C0,I2C_Event_Start,ENABLE);	
		if (!i2c_wait_flag_set(I2C_FLAG_StartOk, 5)) goto fail;
	I2C_SendAddr(I2C0, DeviceAddr);//器件地址，写
	if (!i2c_wait_flag_set(I2C_FLAG_MASGetAckW, 5)) goto fail;
	I2C_SendData(I2C0,WriteAddr);//发送要写的字地址
	if (!i2c_wait_flag_set(I2C_FLAG_MDSGetAck, 5)) goto fail;
	for(i=0;i<data_size;i++)
	{
		I2C_SendData(I2C0, *(pBuffer++));
		if (!i2c_wait_flag_set(I2C_FLAG_MDSGetAck, 5)) goto fail;
	}
	/******************发送停止位***************/
	I2C_GenerateEvent(I2C0,I2C_Event_Stop,ENABLE);
    return;          // ★ 成功直接返回
    fail:
    i2c_recover();
}
uint8_t MPU_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    I2C_EE_Read(&val, reg, MPU_ADDR, 1);
    return val;
}
void MPU_ReadRegs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    I2C_EE_Read(buf, start_reg, MPU_ADDR, len);
}
void MPU_WriteReg(uint8_t reg, uint8_t val)
{
    I2C_EE_Write(&val, reg, MPU_ADDR, 1);
}
int16_t be16_to_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}
void gyro_first_read(void)
{
    // 打印提示信息
	// UART_SendData(UART1, 'g');
	// while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};
	// UART_SendData(UART1, ':');
	// while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};
	// 1) WHO_AM_I
    uint8_t who = MPU_ReadReg(REG_WHO_AM_I);
	UART_SendData(UART1, who);
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};

    // 2) 唤醒（SLEEP=0），最小配置：PWR_MGMT_1 = 0x00
    MPU_WriteReg(REG_PWR_MGMT_1, 0x00);

	delay_ms(10);// TODO: 短暂延时等待硬件准备就绪是否必要？

    who = MPU_ReadReg(REG_WHO_AM_I);
	UART_SendData(UART1, who);
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};
}

// ===== 陀螺仪角度输出（最小可用版：整数近似）=====

static int16_t s_roll_deg = 0;   // -90~+90
static int16_t s_pitch_deg = 0;  // -90~+90
static int16_t s_yaw_deg = 0;    // -90~+90（演示用积分）

static uint32_t isqrt32(uint32_t x)
{
    // 整数平方根（很小的实现）
    uint32_t op = x;
    uint32_t res = 0;
    uint32_t one = 1uL << 30;
    while (one > op) one >>= 2;
    while (one != 0) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return res;
}

// 近似 atan2(y,x) 输出“角度(度)”，范围约 -180~+180
// 说明：这是一个轻量近似，足够用于roll/pitch显示；后续可升级CORDIC/查表
static int16_t atan2_deg_approx(int32_t y, int32_t x)
{
    if (x == 0) {
        return (y > 0) ? 90 : (y < 0 ? -90 : 0);
    }

    // 使用一个常见近似：atan(z) ≈ z*57.3 / (1 + 0.28*z^2)，z = y/x
    // 为避免浮点：用Q12固定点
    int32_t abs_y = (y < 0) ? -y : y;
    int32_t abs_x = (x < 0) ? -x : x;

    // 保证 |z|<=1：用倒数处理
    uint8_t invert = 0;
    int32_t num = abs_y, den = abs_x;
    if (abs_y > abs_x) { invert = 1; num = abs_x; den = abs_y; }

    // z_q12 = num/den
    int32_t z_q12 = (num << 12) / (den ? den : 1);
    // z^2_q12 = (z_q12*z_q12)>>12
    int32_t z2_q12 = (z_q12 * z_q12) >> 12;

    // denom = 1 + 0.28*z^2  (0.28≈72/256)
    int32_t denom_q12 = (1 << 12) + ((72 * z2_q12) >> 8);

    // atan(z) ≈ z * 57.3 / denom
    // 57.3≈ (57.3*4096)=234701 => 用234701近似
    int32_t angle_q12 = (z_q12 * 234701) / (denom_q12 ? denom_q12 : 1);
    int16_t angle = (int16_t)(angle_q12 >> 12);

    if (invert) angle = 90 - angle;

    // 恢复象限
    if (x < 0) angle = 180 - angle;
    if (y < 0) angle = -angle;

    // 归一到 -180..180
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;

    return angle;
}

static int16_t clamp90(int16_t a)
{
    if (a > 90) return 90;
    if (a < -90) return -90;
    return a;
}

static uint8_t map_angle_u8(int16_t deg)
{
    deg = clamp90(deg);
    return (uint8_t)(deg + 90); // -90->0, 0->90, +90->180
}

void gyro_update_20ms(void)
{
    uint8_t buf[14];
    MPU_ReadRegs(REG_ACCEL_XOUT_H, buf, 14);   // 读ACC/T/GYRO（你已有此函数）:contentReference[oaicite:4]{index=4}

    int16_t ax = be16_to_i16(buf[0],  buf[1]);
    int16_t ay = be16_to_i16(buf[2],  buf[3]);
    int16_t az = be16_to_i16(buf[4],  buf[5]);
    int16_t gz = be16_to_i16(buf[12], buf[13]);

    // roll = atan2(ay, az)
    s_roll_deg = clamp90(atan2_deg_approx((int32_t)ay, (int32_t)az));

    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    uint32_t denom = isqrt32((uint32_t)((int32_t)ay * ay + (int32_t)az * az));
    s_pitch_deg = clamp90(atan2_deg_approx((int32_t)(-ax), (int32_t)denom));

    // yaw：用gz积分（演示版），默认±250dps约131 LSB/dps；dt=20ms
    // delta_deg ≈ gz/131 * 0.02
    int32_t delta = ((int32_t)gz * 20) / (131 * 1000); // 很粗的整数积分
    s_yaw_deg = clamp90((int16_t)(s_yaw_deg + delta));
}

void gyro_get_mapped_angles(uint8_t* r, uint8_t* p, uint8_t* y)
{
    if (r) *r = map_angle_u8(s_roll_deg);
    if (p) *p = map_angle_u8(s_pitch_deg);
    if (y) *y = map_angle_u8(s_yaw_deg);
}
