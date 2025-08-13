/** 
  ******************************************************************************
  * @file    stm32l0xx_nucleo_ihm13a1.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    February 23, 2015
  * @brief   Header for BSP driver for x-nucleo-ihm13a1 Nucleo extension board 
  *  (based on Stspin250)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L0XX_NUCLEO_IHM13A1_H
#define __STM32L0XX_NUCLEO_IHM13A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_nucleo.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STM32L0XX_NUCLEO_IHM13A1
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup IHM13A1_Exported_Constants IHM13A1 Exported Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_STM32L0XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_STM32L0XX_NUCLEO   Constants For STM32L0XX NUCLEO
* @{
*/   
/// Interrupt line used for Stspin250 Fault interrupt
#define EXTI_FAULT_IRQn          (EXTI4_15_IRQn)
   
/// Timer used for PWMA
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE      (TIM22)


   /// Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF      (TIM2)

/// Channel Timer used for PWMA
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_BRIDGE      (TIM_CHANNEL_1)

/// Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_REF     (TIM_CHANNEL_1)   

/// HAL Active Channel Timer used for PWMA
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_BRIDGE      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF      (HAL_TIM_ACTIVE_CHANNEL_1)

/// Timer Clock Enable for PWMA
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE_CLCK_ENABLE()  __TIM22_CLK_ENABLE()


/// Timer Clock Enable for REF
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_ENABLE()   __TIM2_CLK_ENABLE()

/// Timer Clock Enable for PWMA
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE_CLCK_DISABLE()  __TIM22_CLK_DISABLE()


   /// Timer Clock Enable for REF
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_DISABLE()  __TIM2_CLK_DISABLE()

/// PWMA GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_BRIDGE  (GPIO_AF4_TIM22)

/// REF GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_REF  (GPIO_AF2_TIM2)   

/// GPIO Pin used for the ref pin of the Stspin250
#define BSP_MOTOR_CONTROL_BOARD_REF_PIN  (GPIO_PIN_0)
/// GPIO Port used for the ref pin of the Stspin250
#define BSP_MOTOR_CONTROL_BOARD_REF_PORT  (GPIOA)
   
 /**
* @}
*/

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

   /** @defgroup Constants_For_All_Nucleo_Platforms Constants For All Nucleo Platforms
* @{
*/   

/// GPIO Pin used for the  input PWM A of the Stspin250 
#define BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PIN  (GPIO_PIN_4)
/// GPIO Port sed for the  input PWM A of the Stspin250 
#define BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PORT  (GPIOB)

/// GPIO Pin used for the direction of the Stspin250 Brige A
#define BSP_MOTOR_CONTROL_BOARD_DIR_A_PIN  (GPIO_PIN_10)
/// GPIO Port used for the direction of the Stspin250 Brige A
#define BSP_MOTOR_CONTROL_BOARD_DIR_A_PORT  (GPIOB)

/// GPIO Pin used for the direction of the Stspin250 Brige B
#define BSP_MOTOR_CONTROL_BOARD_DIR_B_PIN  (GPIO_PIN_8)
/// GPIO Port used for the direction of the Stspin250 Brige B
#define BSP_MOTOR_CONTROL_BOARD_DIR_B_PORT  (GPIOA)
   
/// GPIO Pin used for the Stspin250  Enable pin and Faults (over current detection and thermal shutdown)
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN  (GPIO_PIN_10)
/// GPIO port used for the Stspin250  Enable pin and Faults (over current detection and thermal shutdown)
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT (GPIOA)
/// Flag interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PRIORITY  (3)

/// GPIO Pin used for the standy/reset pin of the Stspin250
#define BSP_MOTOR_CONTROL_BOARD_RESET_PIN  (GPIO_PIN_7)
/// GPIO Port used for the standy/reset pin of the Stspin250
#define BSP_MOTOR_CONTROL_BOARD_RESET_PORT  (GPIOC)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L0XX_NUCLEO_IHM13A1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
