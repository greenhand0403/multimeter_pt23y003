 /******************************************************************************
  * @file    PT32Y003x_it.h
  * @author  应用开发团队
  * @version V1.0.0
  * @date    2022/9/1
  * @brief    This file contains all the functions prototypes for the IT firmware library.
  *          
  ******************************************************************************
  * @attention
  *
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PT32Y003x_it_H
#define PT32Y003x_it_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "PT32Y003x.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SysTick_Handler(void);
void EXTIC_Handler(void);
void TIM1_Handler(void);
void TIM0_Handler(void);
void TIM4_Handler(void);
void TIM2_Handler(void);


/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif

