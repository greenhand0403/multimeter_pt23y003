// main.c
#include "system_config.h"
#include <PT32Y003x_pwr.h>
#include <PT32Y003x_exti.h>
#include <PT32Y003x_gpio.h>
#include "delay.h"
#include "protocol.h"
#include "bluetooth_driver.h"
#include "gyro_driver.h"

#define DEBUG_FLAG 1

// ===== 全局变量定义（业务层）=====
volatile work_mode_t g_work_mode = WORK_MODE_IDLE;
volatile work_mode_t g_work_mode_prev = WORK_MODE_IDLE;

// ===== 外部函数声明 =====
extern void Debug_Printf(const char *format, ...);
extern void Bluetooth_Printf(const char *format, ...);

// 来自各驱动
extern void led_init(void), led_set_on(void), led_set_off(void), led_set_blink_fast(void), led_update(void);
extern void button_init(void);
extern uint8_t button_get_state(void);
extern void bluetooth_init(void), bluetooth_configure_name_start(void);
extern uint8_t BLE_NAME_LEGAL;
extern void uart1_init(void);
extern void SysTick_Init(void);
extern void enter_sleep_mode(void);
extern void PollAndProcessUARTLines(void); // 应移到 bluetooth_driver.c，此处暂留

// 来自 protocol
extern void send_key_status_packet(uint8_t key_state);
extern void send_gyro_data_packet(int16_t gx, int16_t gy, int16_t gz);
extern volatile uint32_t g_last_packet_time;

// ===== 辅助函数 =====
void UART_SendString(UART_TypeDef* UARTx, const char *str) { /* ... */ }
void Bluetooth_Printf(const char *format, ...) { /* ... */ }
void Debug_Printf(const char *format, ...) { /* ... */ }

work_mode_t detect_work_mode(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(GPIOA, &gpio);
    delay_ms(20);
    return (GPIO_ReadDataBit(GPIOA, GPIO_Pin_3) == 1) ? 
           WORK_MODE_1_SERVO : WORK_MODE_2_GYRO;
}

void uart1_init(void) { /* ... */ }

void system_init(void)
{
    SysTick_Init();
#ifdef DEBUG_FLAG
    uart1_init();
#endif
    led_init();
    led_set_blink_fast();
    button_init();
    bluetooth_init();

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_In;
    gpio.GPIO_Pull = GPIO_Pull_NoPull;
    GPIO_Init(GPIOD, &gpio);
}

void enter_sleep_mode(void)
{
    // ... 你的代码 ...
}

void PollAndProcessUARTLines(void)
{
    // 注意：这个函数应移到 bluetooth_driver.c，并由 UART 中断触发
    // 此处为兼容暂留
    extern volatile uint8_t rx_ring[], rx_head, rx_tail;
    static char line_buf[128];
    static uint16_t line_len = 0;

    while (rx_tail != rx_head) {
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % 128;

        if (line_len < sizeof(line_buf) - 1) {
            line_buf[line_len++] = (char)b;
        } else {
            line_len = 0;
        }

        if (b == '\n') {
            if (line_len >= 2 && line_buf[line_len-2] == '\r')
                line_buf[line_len-2] = '\0';
            else
                line_buf[line_len-1] = '\0';

            ProcessBluetoothResponse(line_buf);
            line_len = 0;
        }
    }
}

// ===== 主函数 =====
int main(void)
{
    system_init();

    // 等待蓝牙名称合法
    while (!BLE_NAME_LEGAL) {
        led_update();
        PollAndProcessUARTLines();
        bluetooth_configure_name_start();
        delay_ms(100);
    }
    Debug_Printf("BLE NAME OK\r\n");

    while (1) {
        PollAndProcessUARTLines();

        if (GPIO_ReadDataBit(BT_CONNECT_LED_PIN) == 1) {
            if (g_bt_state == BT_STATE_DISCONNECTED) {
                led_set_on();
                g_bt_state = BT_STATE_CONNECTED;
                send_connect_packet(); // 使用 protocol.c
                g_work_mode_prev = WORK_MODE_IDLE;
            }

            g_work_mode = detect_work_mode();
            if (g_work_mode_prev == WORK_MODE_IDLE) {
                if (g_work_mode == WORK_MODE_1_SERVO) {
                    button_init();
                } else if (g_work_mode == WORK_MODE_2_GYRO) {
                    gyro_init();
                }
                g_work_mode_prev = g_work_mode;
            }

            if (g_work_mode == g_work_mode_prev) {
                if (g_work_mode == WORK_MODE_1_SERVO) {
                    if (s_ms_ticks - g_last_packet_time >= PACKET_SEND_INTERVAL_MS) {
                        uint8_t keys = button_get_state();
                        send_key_status_packet(keys);
                        g_last_packet_time = s_ms_ticks;
                    }
                } else if (g_work_mode == WORK_MODE_2_GYRO) {
                    // request_gyro_data();
                    if (s_ms_ticks - g_last_packet_time >= PACKET_SEND_INTERVAL_MS) {
                        // 假数据
                        send_gyro_data_packet(0x1234, 0x5678, 0x9ABC);
                        g_last_packet_time = s_ms_ticks;
                    }
                }
            } else {
                g_work_mode_prev = WORK_MODE_IDLE;
            }
        } else {
            if (g_bt_state == BT_STATE_CONNECTED) {
                led_set_blink_fast();
                g_bt_state = BT_STATE_DISCONNECTED;
            }
        }

        led_update();
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