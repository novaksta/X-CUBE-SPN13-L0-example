/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM13A1_ExampleFor1BiDirMotor/Src/stm32l0xx_hal_msp.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    August 16, 2016
  * @brief   HAL MSP module.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void BSP_MotorControl_FlagInterruptHandler(void);
extern void ButtonHandler(void);
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief PWM MSP Initialization 
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE)&&
      (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_BRIDGE))                 
  {  
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE_CLCK_ENABLE();
    
    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_BRIDGE;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PORT, &GPIO_InitStruct);
   }
  else if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF)&&
           (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF))           
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_ENABLE();
  
    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_REF_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_REF;    
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_REF_PORT, &GPIO_InitStruct);    
  }
}

/**
  * @brief PWM MSP De-Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
   GPIO_InitTypeDef  GPIO_InitStruct;
 
 if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE)&&
       (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_BRIDGE))
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_BRIDGE_CLCK_DISABLE();
  
    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PIN);
    
    // Reconfigure PWMA pin as output pull down
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_BRIDGE_PIN, GPIO_PIN_RESET);
    
  }  
  else if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF)&&
           (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF))
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_DISABLE();
  
    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_REF_PORT, BSP_MOTOR_CONTROL_BOARD_REF_PIN);
  }
}

/**
  * @brief External Line Callback 
  * @param[in] GPIO_Pin pin number
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN)
  {
    BSP_MotorControl_FlagInterruptHandler();
  }

  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    ButtonHandler();
  }  
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
