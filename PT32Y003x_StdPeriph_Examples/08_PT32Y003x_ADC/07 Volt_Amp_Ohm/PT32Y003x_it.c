/******************************************************************************
  * @file    PT32Y003x_it.c
  * @author  应用开发团队
  * @version V1.6.0
  * @date    2023/12/18
  * @brief    This file provides all interrupt service routine.
  *          
  ******************************************************************************
  * @attention
  *
  *
  *****************************************************************************/
  
/* Includes ------------------------------------------------------------------------------------------------*/
#include "PT32Y003x_it.h"
#include <PT32Y003x_tim.h>
#include <PT32Y003x_gpio.h>

/** @defgroup IT
  * @brief IT driver modules
  * @{
  */
  
/* Private typedef -----------------------------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------------------------------------*/

extern volatile uint32_t s_ms_ticks;   // 1ms 计数（全局）
extern volatile uint32_t s_ms_delay;   // 阻塞式 ms 延时用
// SysTick 中断：1ms 心跳 + 阻塞延时递减
/**
* @brief SysTick中断服务函数
* @param None
* @retval None
*/
void SysTick_Handler(void)
{
  s_ms_ticks++;
  if (s_ms_delay) s_ms_delay--;
}

/**
* @brief PC中断服务函数
* @param None
* @retval None
*/
void EXTIC_Handler(void)
{
  EXTI_ClearFlag(EXTIC, GPIO_Pin_5);
}

/**
* @brief TIMER1中断服务函数
* @param None
* @retval None
*/
void TIM1_Handler(void)
{
	
}

/**
* @brief TIMER2中断服务函数
* @param None
* @retval None
*/

extern volatile uint8_t g_short_press_event;
extern const uint32_t PWR_DEBOUNCE_MS;
extern const uint32_t PWR_LONGPRESS_MS;
extern const uint32_t POSTWAKE_LONGPRESS_TIMEOUT;
// 由 TIM2 每 10ms 扫描用到的计数
extern volatile uint16_t s_pwr_stable_ticks;
extern volatile uint16_t s_pwr_press_ticks;
extern volatile uint8_t  s_pwr_last_sample;   // 1=未按, 0=按下
extern volatile uint8_t  poweroff_request;   // 置 1 后在安全点 deep_sleep()
// 运行模式：工作态 唤醒态 等待态
typedef enum { RUN_MODE_NORMALWORK = 0, RUN_MODE_DEEPSLEEP = 1, RUN_MODE_WAKEUP = 2} run_mode_t;
extern run_mode_t g_run_mode;
// 全局（或静态）加一个锁
uint8_t s_lock_until_release = 0;
void TIM2_Handler(void)
{
  if (TIM_GetFlagStatus(TIM2, TIM_FLAG_ARF) != RESET)
  {
    TIM_ClearFlag(TIM2, TIM_FLAG_ARF);

    const uint16_t DEBOUNCE_TICKS  = (uint16_t)(PWR_DEBOUNCE_MS / 10U);
    const uint16_t LONGPRESS_TICKS = (uint16_t)(PWR_LONGPRESS_MS / 10U);

    uint8_t sample = GPIO_ReadDataBit(GPIOC, GPIO_Pin_5);

    if (sample == s_pwr_last_sample) {
      if (s_pwr_stable_ticks < DEBOUNCE_TICKS)
        s_pwr_stable_ticks++;
    } else {
      s_pwr_stable_ticks = 0;
      s_pwr_last_sample = sample;
    }

    uint8_t pressed_stable = (s_pwr_last_sample == 0) && (s_pwr_stable_ticks >= DEBOUNCE_TICKS);

    if (s_lock_until_release) {
      if (pressed_stable) {
        return;
      } else {
        s_lock_until_release = 0;
        s_pwr_press_ticks = 0;
        return;
      }
    }

    if (pressed_stable) {
      if (s_pwr_press_ticks < LONGPRESS_TICKS)
        s_pwr_press_ticks++;
    } else {
      // 松手瞬间判定短按
      if (s_pwr_press_ticks >= DEBOUNCE_TICKS && s_pwr_press_ticks < LONGPRESS_TICKS) {
        if (g_run_mode == RUN_MODE_NORMALWORK) {
          g_short_press_event = 1;
        }
      }
      s_pwr_press_ticks = 0;
    }

    if (s_pwr_press_ticks >= LONGPRESS_TICKS) {
      s_pwr_press_ticks = 0;
      if (g_run_mode == RUN_MODE_NORMALWORK) {
        poweroff_request = 1;
      } else {
        g_run_mode = RUN_MODE_NORMALWORK;
      }
      s_lock_until_release = 1;
    }
  }
}

/**
* @brief TIMER3中断服务函数
* @param None
* @retval None
*/
void TIM3_Handler(void)
{
}

/**
* @brief TIMER4中断服务函数
* @param None
* @retval None
*/
// extern uint32_t mcu_sleep_count;
void TIM4_Handler(void)
{
  // 120秒自动休眠 弃用 改用systick记录120s休眠了
  // TIM_ClearFlag(TIM4, TIM_FLAG_ARF);
  // mcu_sleep_count += 5;
}
/**
  * @}
  */

