// gyro_driver.c
#include "gyro_driver.h"
#include "delay.h"

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
void I2C_EE_Read(uint8_t* pBuffer,uint16_t ReadAddr, uint16_t DeviceAddr, uint16_t data_size)
{
	int i;
/******************等待从机ready***************/		
	I2C_GenerateEvent(I2C0,I2C_Event_Start,ENABLE);
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_StartOk)!= SET);
	I2C_SendAddr(I2C0, DeviceAddr);//器件地址，写
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MASGetAckW)!=SET);
	I2C_SendData(I2C0,ReadAddr);//发送要读的页地址
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MDSGetAck)!=SET);
	I2C_GenerateEvent(I2C0,I2C_Event_Stop,ENABLE);
/******************接收数据***************/		
	I2C_GenerateEvent(I2C0,I2C_Event_Start,ENABLE);
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_StartOk)!= SET);
	I2C_SendAddr(I2C0, DeviceAddr|0x1);//器件地址，读
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MASGetAckR)!=SET);
	I2C0->CR=I2C_CR_ACK;
	for(i=0;i<data_size;i++)
	{
		if(i==(data_size-1))
		{
			I2C0->CCR=I2C_CCR_ACK|I2C_CCR_SI;//主机发送NACK
			while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MDGSendNack) != SET);
			*pBuffer=I2C_ReceiveData(I2C0);
			pBuffer++;
			break;
		}
		I2C0->CCR=I2C_CCR_SI;
		while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MDGSendAck) != SET);
		*pBuffer=I2C_ReceiveData(I2C0);
		pBuffer++;
	}
	/******************发送停止位***************/
	I2C_GenerateEvent(I2C0,I2C_Event_Stop,ENABLE);
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
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_StartOk)!= SET);
	I2C_SendAddr(I2C0, DeviceAddr);//器件地址，写
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MASGetAckW)!=SET);
	I2C_SendData(I2C0,WriteAddr);//发送要写的字地址
	while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MDSGetAck)!=SET);
	for(i=0;i<data_size;i++)
	{
		I2C_SendData(I2C0, *(pBuffer++));
		while(I2C_GetFlagStatus(I2C0,I2C_FLAG_MDSGetAck) != SET);
	}
	/******************发送停止位***************/
	I2C_GenerateEvent(I2C0,I2C_Event_Stop,ENABLE);
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
	UART_SendData(UART1, 'g');
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};
	UART_SendData(UART1, ':');
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};
	// 1) WHO_AM_I
    uint8_t who = MPU_ReadReg(REG_WHO_AM_I);
	UART_SendData(UART1, who);
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};

    // 2) 唤醒（SLEEP=0），最小配置：PWR_MGMT_1 = 0x00
    MPU_WriteReg(REG_PWR_MGMT_1, 0x00);

	delay_ms(10);

    who = MPU_ReadReg(REG_WHO_AM_I);
	UART_SendData(UART1, who);
	while (!UART_GetFlagStatus(UART1, UART_FLAG_TXE)){};

	// uint8_t buf[14];
	// 连续读 14 字节：ACCEL(6) + TEMP(2) + GYRO(6)
	// MPU_ReadRegs(REG_ACCEL_XOUT_H, buf, 14);

	// int16_t ax = be16_to_i16(buf[0],  buf[1]);
	// int16_t ay = be16_to_i16(buf[2],  buf[3]);
	// int16_t az = be16_to_i16(buf[4],  buf[5]);
	// int16_t t  = be16_to_i16(buf[6],  buf[7]);
	// int16_t gx = be16_to_i16(buf[8],  buf[9]);
	// int16_t gy = be16_to_i16(buf[10], buf[11]);
	// int16_t gz = be16_to_i16(buf[12], buf[13]);

	// LOGF("A[%d,%d,%d]  G[%d,%d,%d]  Traw=%d\r\n", ax, ay, az, gx, gy, gz, t);
}