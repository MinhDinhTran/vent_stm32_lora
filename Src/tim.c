/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* TIM2 init function */
void MX_TIM2_Init(void)
{
/* USER CODE BEGIN 2 */
/*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);;
  
  /* GPIO TIM3_CH1 configuration */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);

  /***********************************************/
  /* Configure the NVIC to handle TIM3 interrupt */
  /***********************************************/
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
  
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the peripheral clock of TIM3 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Select counter mode: counting up */
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  
  /* Set the one pulse mode:  generate only 1 pulse*/
  LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
  
  /* Set the TIM3 prescaler to get counter clock frequency at 10 MHz          */
  /* In this example TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK2), */
  /* since APB1 pre-scaler is equal to 1.                                     */
  /*    TIM3CLK = PCLK2                                                       */
  /*    PCLK2 = HCLK                                                          */
  /*    => TIM3CLK = SystemCoreClock (72 MHz)                                 */  
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock, 10000000));
  
  /* Set the capture/compare register to get a pulse delay of 50 us */
  LL_TIM_OC_SetCompareCH1(TIM2, __LL_TIM_CALC_DELAY(SystemCoreClock, LL_TIM_GetPrescaler(TIM2), 5800)); 
  
  /* Set the autoreload register to get a pulse length of 50s */
  LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_PULSE(SystemCoreClock, LL_TIM_GetPrescaler(TIM2), 5800, 50)); // REG 2800 MAX speed - 5800 min speed

  /* Set output channel 1 in PWM2 mode */
  LL_TIM_OC_SetMode(TIM2,  LL_TIM_CHANNEL_CH1,  LL_TIM_OCMODE_PWM2);

  /* Configure output channel 1 */
  LL_TIM_OC_ConfigOutput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_LOW);
  
  /**************************/
  /* TIM3 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1 */
  LL_TIM_EnableIT_CC1(TIM2);
  
  /**************************/
  /* Start pulse generation */
  /**************************/
  /* Enable channel 1 */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  
  /* Enable TIM3 outputs */
  LL_TIM_EnableAllOutputs(TIM2);
  
  /* Enable auto-reload register preload */
  LL_TIM_EnableARRPreload(TIM2);

  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM2);  
/* USER CODE END 2 */

  
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
