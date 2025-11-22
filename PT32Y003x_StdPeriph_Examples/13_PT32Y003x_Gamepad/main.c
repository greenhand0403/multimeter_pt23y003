#include "system_config.h"
#include <PT32Y003x_pwr.h>
#include <PT32Y003x_exti.h>
// 头文件的添加在keil里面设置了，自动搜索头文件

// ===== 全局变量 =====
volatile uint8_t g_current_key_state = 0;
volatile work_mode_t g_work_mode = WORK_MODE_IDLE;
extern volatile bluetooth_state_t g_bt_state;
volatile uint8_t g_seq_num = 0;
volatile uint32_t g_last_packet_time = 0;

// ===== 外部函数声明 =====
extern void led_init(void);
extern void led_set_blink_fast(void);  // 只保留快闪
extern void led_set_on(void);
extern void led_set_off(void);
extern void led_update(void);

extern void button_init(void);
extern uint8_t button_get_state(void);
extern void button_check_wakeup(void);

extern void servo_init(void);
extern void servo_set_angle(uint8_t angle);

extern void bluetooth_init(void);
extern void bluetooth_send_packet(protocol_packet_t* packet);
extern void bluetooth_check_connection(void);
extern uint8_t bluetooth_check_sleep_timeout(void);
extern void bluetooth_configure_name(void);

extern void gyro_init(void);

// ===== 模式检测函数 =====
work_mode_t detect_work_mode(void)
{
    GPIO_InitTypeDef gpio;
    
    // 配置PA3为输入上拉
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOA, &gpio);
    
    delay_ms(10);
    
    if (GPIO_ReadDataBit(GPIOA, GPIO_Pin_3) == 1) {
        return WORK_MODE_1_SERVO;  // 悬空 - 模式1
    } else {
        return WORK_MODE_2_GYRO;   // 接地 - 模式2
    }
}

// ===== 初始化所有模块 =====
void system_init(void)
{
    SysTick_Init();
    
    // 检测工作模式
    g_work_mode = detect_work_mode();
    
    // 初始化各模块
    led_init();
    button_init();
    bluetooth_init();
    
    // 根据模式初始化相应模块
    if (g_work_mode == WORK_MODE_1_SERVO) {
        servo_init();
    } else if (g_work_mode == WORK_MODE_2_GYRO) {
        gyro_init();
    }
}

// ===== 发送连接状态包 =====
void send_connect_packet(void)
{
    protocol_packet_t packet;
    
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_CONNECT;
    
    // MAC地址后2字节（实际应从蓝牙模块读取）
    packet.data[0] = 0xAA;
    packet.data[1] = 0xBB;
    packet.data[2] = 0x00;
    packet.data[3] = 0x00;
    packet.data[4] = 0x00;
    packet.data[5] = 0x00;
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + (packet.data[0] + packet.data[1] + 
                packet.data[2] + packet.data[3] + packet.data[4] + 
                packet.data[5]) + packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
}

// ===== 发送按键状态包 =====
void send_key_status_packet(void)
{
    protocol_packet_t packet;
    
    packet.header_h = PROTOCOL_HEADER_H;
    packet.header_l = PROTOCOL_HEADER_L;
    packet.cmd_type = CMD_TYPE_STATUS;
    
    packet.data[0] = g_current_key_state;
    packet.data[1] = 0x00;
    packet.data[2] = 0x00;
    // 陀螺仪数据（模式2时使用）
    packet.data[3] = 0x5A;
    packet.data[4] = 0x5A;
    packet.data[5] = 0x5A;
    
    packet.seq_num = g_seq_num++;
    
    uint16_t crc = packet.cmd_type + (packet.data[0] + packet.data[1] + 
                packet.data[2] + packet.data[3] + packet.data[4] + 
                packet.data[5]) + packet.seq_num;
    packet.crc_high = (uint8_t)(crc >> 8);
    packet.crc_low = (uint8_t)(crc & 0xFF);
    
    packet.tail_h = PROTOCOL_TAIL_H;
    packet.tail_l = PROTOCOL_TAIL_L;
    
    bluetooth_send_packet(&packet);
}

// ===== 休眠功能 =====
void enter_sleep_mode(void)
{
    led_set_off();
    
    // 配置任意按键唤醒
    EXTI_TriggerTypeConfig(EXTIA, GPIO_Pin_1, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIA, GPIO_Pin_2, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_3, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_4, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_5, EXTI_Trigger_RisingFalling);
    EXTI_TriggerTypeConfig(EXTIC, GPIO_Pin_6, EXTI_Trigger_RisingFalling);
    
    EXTI_ITConfig(EXTIA, GPIO_Pin_1, ENABLE);
    EXTI_ITConfig(EXTIA, GPIO_Pin_2, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_3, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_4, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_5, ENABLE);
    EXTI_ITConfig(EXTIC, GPIO_Pin_6, ENABLE);
    
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = EXTIA_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x00;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = EXTIC_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPriority = 0x00;
    NVIC_Init(&nvic);
    
    PWR_EnterDeepSleepMode(PWR_DeepSleepEntry_WFI);
}

// ===== 主函数 =====
int main(void)
{
    system_init();
    
    // 配置蓝牙名称
    bluetooth_configure_name();
    
    // 发送连接确认包
    send_connect_packet();
    
    while (1)
    {
        bluetooth_check_connection();
        
        // LED控制：严格按照设计文档
        if (g_bt_state == BT_STATE_CONNECTED) {
            led_set_on();      // 连接后常亮
        } else {
            led_set_blink_fast();  // 断开后快闪（唯一闪烁模式）
        }
        
        // 获取按键状态
        uint8_t new_key_state = button_get_state();
        
        // 数据发送逻辑
        uint32_t current_time = s_ms_ticks;
        if ((new_key_state != g_current_key_state) || 
            (g_work_mode == WORK_MODE_2_GYRO && 
             (current_time - g_last_packet_time) >= PACKET_SEND_INTERVAL_MS)) {
            
            g_current_key_state = new_key_state;
            g_last_packet_time = current_time;
            
            if (g_bt_state == BT_STATE_CONNECTED) {
                send_key_status_packet();
            }
        }
        
        led_update();
        
        // 检查休眠超时
        if (bluetooth_check_sleep_timeout()) {
            enter_sleep_mode();
        }
        
        delay_ms(1);
    }
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