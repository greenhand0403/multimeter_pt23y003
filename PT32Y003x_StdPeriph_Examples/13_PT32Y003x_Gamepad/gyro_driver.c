// gyro_driver.c
#include "gyro_driver.h"
#include "delay.h"
#include <math.h>
// ===== 陀螺仪角度输出（最小可用版：整数近似）=====

static int16_t s_roll_deg = 0;   // -90~+90
static int16_t s_pitch_deg = 0;  // -90~+90
static int16_t s_yaw_deg = 0;    // -90~+90（演示用积分）

static int32_t s_yaw_mdeg = 0; // 用于积分解决Z轴测量范围不对、有零漂的问题
static int32_t s_gz_bias = 0;
static uint32_t s_last_yaw_ms = 0;
static uint32_t s_a2_ref = 0;

static uint16_t s_mpu_fail_cnt = 0;

// ===== roll/pitch：用陀螺积分维持连续性（单位：毫度）=====
static int32_t s_roll_mdeg = 0;
static int32_t s_pitch_mdeg = 0;
static uint32_t s_last_rp_ms = 0;

// 陀螺零偏（原始LSB）
static int32_t s_gx_bias = 0;
static int32_t s_gy_bias = 0;

// 退化区阈值：在 pitch 接近 ±90° 时，az≈0，roll 的 accel 观测会退化
static const int16_t AZ_DEADZONE = 1000;      // |az| < 该值：认为roll accel观测不可靠
static const int32_t DENOM_DEADZONE = 1500;   // sqrt(ay^2+az^2) < 该值：pitch accel观测不可靠

// 互补滤波系数：98% 信任陀螺，2% 用加速度纠偏（可调：97/3 或 99/1）
static const int32_t CF_GYRO_W = 98;
static const int32_t CF_ACC_W  = 2;

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
    // delay_us(200);
    I2C_Cmd(I2C0, ENABLE);
}
uint8_t I2C_EE_Read(uint8_t* pBuffer,uint16_t ReadAddr, uint16_t DeviceAddr, uint16_t data_size)
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
    return 1;          // ★ 成功直接返回
fail:
    I2C_GenerateEvent(I2C0, I2C_Event_Stop, ENABLE); // 关键：失败也发 STOP
    i2c_recover();
    return 0;
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
    I2C_GenerateEvent(I2C0, I2C_Event_Stop, ENABLE);
    i2c_recover();
}
uint8_t MPU_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    I2C_EE_Read(&val, reg, MPU_ADDR, 1);
    return val;
}
// 读取连续的多个地址
uint8_t MPU_ReadRegs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    return I2C_EE_Read(buf, start_reg, MPU_ADDR, len);
}
void MPU_WriteReg(uint8_t reg, uint8_t val)
{
    I2C_EE_Write(&val, reg, MPU_ADDR, 1);
}
int16_t be16_to_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}
static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

// 只读取gz（通过连续读0x3B的14字节，保持与你现有读取方式一致）
static uint8_t mpu_read_gz(int16_t* out_gz, int16_t* out_ax, int16_t* out_ay, int16_t* out_az)
{
    uint8_t buf[14];
    if (!MPU_ReadRegs(REG_ACCEL_XOUT_H, buf, 14)) return 0;
    int16_t ax = be16_to_i16(buf[0],  buf[1]);
    int16_t ay = be16_to_i16(buf[2],  buf[3]);
    int16_t az = be16_to_i16(buf[4],  buf[5]);
    int16_t gz = be16_to_i16(buf[12], buf[13]);
    if (out_gz) *out_gz = gz;
    if (out_ax) *out_ax = ax;
    if (out_ay) *out_ay = ay;
    if (out_az) *out_az = az;
    return 1;
}
void gyro_first_read(void)
{
	// 1) WHO_AM_I
    uint8_t who = MPU_ReadReg(REG_WHO_AM_I);
	UART_SendData(UART1, who);
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};

    // 2) 唤醒（SLEEP=0），最小配置：PWR_MGMT_1 = 0x00
    MPU_WriteReg(REG_PWR_MGMT_1, 0x00);

	delay_ms(10);// TODO: 短暂延时等待硬件准备就绪是否必要？

    // ===== 启动静止校准：估计 gx/gy 零偏，减少积分漂移 =====
    {
        const uint16_t N = 60;
        const uint16_t DLY = 5;
        int64_t sum_gx = 0, sum_gy = 0;
        uint16_t ok = 0;
        for (uint16_t i = 0; i < N; i++) {
            uint8_t b[14];
            if (MPU_ReadRegs(REG_ACCEL_XOUT_H, b, 14)) {
                int16_t gx = be16_to_i16(b[8],  b[9]);
                int16_t gy = be16_to_i16(b[10], b[11]);
                sum_gx += gx;
                sum_gy += gy;
                ok++;
            }
            delay_ms(DLY);
        }
        if (ok) {
            s_gx_bias = (int32_t)(sum_gx / ok);
            s_gy_bias = (int32_t)(sum_gy / ok);
        } else {
            s_gx_bias = 0;
            s_gy_bias = 0;
        }
        s_roll_mdeg = 0;
        s_pitch_mdeg = 0;
        s_last_rp_ms = s_ms_ticks;
    }
}

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
static void mpu_recover(void)
{
    // 1) 先让I2C尽量回到空闲
    I2C_GenerateEvent(I2C0, I2C_Event_Stop, ENABLE);
    i2c_recover();

    // 2) 软复位MPU（PWR_MGMT_1 bit7 = DEVICE_RESET）
    MPU_WriteReg(REG_PWR_MGMT_1, 0x80);
    delay_ms(50);

    // 3) 唤醒（SLEEP=0）
    MPU_WriteReg(REG_PWR_MGMT_1, 0x00);
    delay_ms(10);

    // 4) 可选：清零yaw积分，避免恢复后突然跳变
    s_yaw_deg = 0;
}
void gyro_update_20ms(void)
{
    uint8_t buf[14];
    if (!MPU_ReadRegs(REG_ACCEL_XOUT_H, buf, 14)) {
        if (++s_mpu_fail_cnt >= 20) {     // 连续失败20次≈400ms(20ms周期)
            s_mpu_fail_cnt = 0;
            mpu_recover();
        }
        return;
    }
    s_mpu_fail_cnt = 0; // 成功一次就清零

    int16_t ax = be16_to_i16(buf[0],  buf[1]);
    int16_t ay = be16_to_i16(buf[2],  buf[3]);
    int16_t az = be16_to_i16(buf[4],  buf[5]);
    int16_t gx = be16_to_i16(buf[8],  buf[9]);
    int16_t gy = be16_to_i16(buf[10], buf[11]);
    int16_t gz = be16_to_i16(buf[12], buf[13]);

    // ===== yaw：用“真实dt(ms)”积分 + gz零偏校准，内部不clamp =====
    uint32_t now = s_ms_ticks;
    uint32_t dt = (s_last_yaw_ms == 0) ? 20 : (now - s_last_yaw_ms);
    s_last_yaw_ms = now;
    if (dt > 100) dt = 100; // 防止长时间卡住导致一次积分过大（保护）
    int32_t gz_corr = (int32_t)gz - s_gz_bias;   // 去零偏后的gz
    int32_t gx_corr = (int32_t)gx - s_gx_bias;
    int32_t gy_corr = (int32_t)gy - s_gy_bias;
// -------------------- roll/pitch：纯加速度（对称抗退化） --------------------
static int32_t ax_f = 0, ay_f = 0, az_f = 0;   // 一阶低通（可选但推荐）

// 1) 简单低通，减少抖动（3/4旧 + 1/4新）
ax_f = (ax_f * 3 + ax) / 4;
ay_f = (ay_f * 3 + ay) / 4;
az_f = (az_f * 3 + az) / 4;

// 2) 对称公式（核心）
// roll  = atan2(ay, sqrt(ax^2 + az^2))
uint32_t denom_r_u = isqrt32((uint32_t)((int32_t)ax_f * ax_f + (int32_t)az_f * az_f));
int32_t denom_r = (int32_t)denom_r_u;
if (denom_r < 1) denom_r = 1;
s_roll_deg = clamp90(atan2_deg_approx((int32_t)ay_f, denom_r));

// pitch = atan2(-ax, sqrt(ay^2 + az^2))
uint32_t denom_p_u = isqrt32((uint32_t)((int32_t)ay_f * ay_f + (int32_t)az_f * az_f));
int32_t denom_p = (int32_t)denom_p_u;
if (denom_p < 1) denom_p = 1;
s_pitch_deg = clamp90(atan2_deg_approx((int32_t)(-ax_f), denom_p));

    // mdeg增量：d(deg) = (gz/131)*(dt/1000) => d(mdeg)= gz*dt/131
    // 这里用毫度积分，分辨率比你原先“整数度”高很多，抖动/跳变会明显变小
    s_yaw_mdeg += (gz_corr * (int32_t)dt) / 131;
    // ===== 在线微调 bias（静止时慢慢贴合）=====
    // 判定“静止”的非常轻量条件：
    // 1) 去偏后的|gz|很小
    // 2) 加速度模长平方与启动参考相近（粗略判断没在大幅运动/震动）
    if (s_a2_ref != 0) {
        uint32_t a2 = (uint32_t)((int32_t)ax * ax) +
                    (uint32_t)((int32_t)ay * ay) +
                    (uint32_t)((int32_t)az * az);
        uint32_t diff = (a2 > s_a2_ref) ? (a2 - s_a2_ref) : (s_a2_ref - a2);
        // 阈值可调：下面两个阈值越严格，越不容易“动的时候被当作静止”
        if (iabs32(gz_corr) < 50 && diff < (s_a2_ref / 20)) { // ~5%窗口
            // IIR：bias = 0.999*bias + 0.001*gz
            s_gz_bias = (s_gz_bias * 999 + (int32_t)gz) / 1000;
        }
    }
}
static uint8_t remap_u8(uint8_t u)
{
    // 你实测范围：min=0x04, max=0xAE
    const int32_t in_min = 0x06;   // 6
    const int32_t in_max = 0xAC;   // 172
    const int32_t out_min = 0;
    const int32_t out_max = 0xB4;  // 180

    int32_t x = (int32_t)u;
    if (x <= in_min) return (uint8_t)out_min;
    if (x >= in_max) return (uint8_t)out_max;

    // 线性拉伸：y = (x-in_min)*(out_range)/(in_range)
    int32_t y = (x - in_min) * (out_max - out_min) / (in_max - in_min);
    if (y < 0) y = 0;
    if (y > 180) y = 180;
    return (uint8_t)y;
}

void gyro_get_mapped_angles(uint8_t* r, uint8_t* p, uint8_t* y)
{
    // 实测发现到不了极限值±90°，所以做一个映射
    if (r) *r = remap_u8(map_angle_u8(s_roll_deg));
    if (p) *p = remap_u8(map_angle_u8(s_pitch_deg));
    // if (y) *y = map_angle_u8(s_yaw_deg);
    if (y) {
        int16_t yaw_deg = (int16_t)(s_yaw_mdeg / 1000);  // 毫度->度（截断）
        yaw_deg = clamp90(yaw_deg);                      // 按你协议只输出-90~+90
        *y = map_angle_u8(yaw_deg);                      // 映射到0~180
    }
}
