/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM13A1_ExampleFor1BiDirMotor/Src/main.c 
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    August 16, 2016
  * @brief   This example shows how to use one X-NUCLEO-IHM13A1 expansion board with 
  * 1 bidirectionnal Brush DC motor.
  * The demo sequence starts when the user button is pressed.
  * Each time, the user button is pressed, the demo step is changed
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

/** @defgroup IHM13A1_Example_for_1_Bidirectionnal_motor
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_STEPS (11)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 static volatile uint16_t gLastError;
 static volatile bool gButtonPressed = FALSE;
 static volatile uint8_t gStep = MAX_STEPS;
 
 Stspin240_250_Init_t gStspin240_250InitParams =
 {
  {20000}, // Frequency of PWM of Input Bridge in Hz up to 100000Hz
  20000,                // Frequency of PWM used for Ref pin in Hz up to 100000Hz
  50,                   //Duty cycle of PWM used for Ref pin (from 0 to 100)
  0                  // Dual Bridge configuration  ( always FALSE for STSPIN250)
 };
   
/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32xx HAL library initialization */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
    /* Configure KEY Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   
  
  
  /* Set Systick Interrupt to the highest priority to have HAL_Delay working*/
  /* under the user button handler */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0); 
  
  //----- Init of the Motor control library 
  /* Set the Stspin240_250 library to use 1 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN250, 1);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the Stspin240_250 library parameters are set with the predefined values from file   */
  /* stspin240_250_target_config.h, otherwise the registers are set using the   */
  /* Stspin240_250_Init_t pointer structure                */
  /* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
  /* device with the structure gStspin240_250InitParams declared in the the main.c file */
  /* and comment the subsequent call having the NULL pointer                   */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN250, &gStspin240_250InitParams);
  //BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN250, NULL);
  
  /* Disable Dual bridge configuration as it is not supported by STSPIN250*/
  BSP_MotorControl_SetDualFullBridgeConfig(0);
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(Error_Handler);
 
  /* Set PWM Frequency of Ref to 15000 Hz */ 
  BSP_MotorControl_SetRefFreq(0,15000); 

  /* Set PWM duty cycle of Ref to 60% */ 
  BSP_MotorControl_SetRefDc(0,60); 
  
  /* Set PWM Frequency of bridge A inputs to 10000 Hz */ 
  BSP_MotorControl_SetBridgeInputPwmFreq(0,10000); 
  
  /* Infinite loop */
  while(1)
  {
    /* Each time the user button is pressed, the step is increased by 1 */
    if (gButtonPressed)
    {
      gButtonPressed = FALSE;
      gStep++;
      if (gStep > MAX_STEPS)
      {
        gStep = 0;
      }
      
      switch (gStep)
      {  
        case 0:
          /*********** Step 0  ************/
          /* Set speed of motor 0 to 100 % */
          BSP_MotorControl_SetMaxSpeed(0,100); 
          /* start motor 0 to run forward*/
          /* if chip is in standby mode */
          /* it is automatically awakened */
          BSP_MotorControl_Run(0, FORWARD);
          break;
      
         case 1:
          /*********** Step 1  ************/
          /* Set speed of motor 0 to 75 % */
          BSP_MotorControl_SetMaxSpeed(0,75); 
          break;
      
        case 2:
          /*********** Step 2 ************/
          /* Set speed of motor 0 to 50 % */
          BSP_MotorControl_SetMaxSpeed(0,50);   
          break;      
      
        case 3:
          /*********** Step 3 ************/
          /* Set speed of motor 0 to 25 % */
          BSP_MotorControl_SetMaxSpeed(0,25);  
          break;  
      
        case 4:
          /*********** Step 4 ************/
          /* Stop Motor 0 */
          BSP_MotorControl_HardStop(0);   
          break;         
         case 5:
          /*********** Step 5  ************/
          /* Set speed of motor 0 to 25 % */
          BSP_MotorControl_SetMaxSpeed(0,25); 
          /* start motor 0 to run backward */
          BSP_MotorControl_Run(0, BACKWARD);
          break;
      
         case 6:
          /*********** Step 6  ************/
          /* Set speed of motor 0 to 50 % */
          BSP_MotorControl_SetMaxSpeed(0,50); 
          break;
      
        case 7:
          /*********** Step 7 ************/
          /* Set speed of motor 0 to 75 % */
          BSP_MotorControl_SetMaxSpeed(0,75);   
          break;      
      
        case 8:
          /*********** Step 8 ************/
          /* Set speed of motor 0 to 100 % */
          BSP_MotorControl_SetMaxSpeed(0,100);   
          break;  
      
        case 9:
          /*********** Step 9 ************/
          /* Stop motor and disable bridge */
          BSP_MotorControl_CmdHardHiZ(0);    

          break;           
        case 10:
          /*********** Step 10 ************/
          /* Start motor to go forward*/
          BSP_MotorControl_Run(0,FORWARD);    
          break;                 
        case 11:
        default:
          /*********** Step 11 ************/
          /* Stop motor and put chip in standby mode */
          BSP_MotorControl_Reset(0);    
          break;            
      }
    } 
  }
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge  */
  uint16_t bridgeState  = BSP_MotorControl_CmdGetStatus(0);
  
  if (bridgeState == 0) 
  {
    if (BSP_MotorControl_GetDeviceState(0) != INACTIVE)
    {
      /* Bridges were disabled due to overcurrent or over temperature */
      /* When  motor was running */
        Error_Handler(0XBAD0);
    }
  }
 }

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  This function is executed when the Nucleo User button is pressed
  * @param  error number of the error
  * @retval None
  */
void ButtonHandler(void)
{
  gButtonPressed = TRUE;
  
  /* Let 300 ms before clearing the IT for key debouncing */
  HAL_Delay(300);
  __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
  HAL_NVIC_ClearPendingIRQ(KEY_BUTTON_EXTI_IRQn);
}    

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
