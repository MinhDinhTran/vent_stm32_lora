/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */    
#include "spi.h"
#include "gpio.h"
#include "SX1278.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId LoRaHandle;
osThreadId FanHandle;
osThreadId mcuWorkHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartLoRa(void const * argument);
void StartTaskFan(void const * argument);
void StartMcuWork(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LoRa */
  osThreadDef(LoRa, StartLoRa, osPriorityAboveNormal, 0, 180);
  LoRaHandle = osThreadCreate(osThread(LoRa), NULL);

  /* definition and creation of Fan */
  osThreadDef(Fan, StartTaskFan, osPriorityNormal, 0, 128);
  FanHandle = osThreadCreate(osThread(Fan), NULL);

  /* definition and creation of mcuWork */
  osThreadDef(mcuWork, StartMcuWork, osPriorityNormal, 0, 128);
  mcuWorkHandle = osThreadCreate(osThread(mcuWork), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartLoRa */
/**
  * @brief  Function implementing the LoRa thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartLoRa */
void StartLoRa(void const * argument)
{

  /* USER CODE BEGIN StartLoRa */
  SX1278_hw_t SX1278_hw;
  SX1278_t SX1278;
  int ret;
  char buffer[64];
  
  
#if defined LORA_MASTER_DEBUG || defined LORA_MASTER_RELEASE
  int message = 0;
  int message_length;
#endif 
  
  
  //initialize LoRa module
  SX1278_hw.dio0.port = IRQ_DIO0_GPIO_Port;
  SX1278_hw.dio0.pin = IRQ_DIO0_Pin;
  SX1278_hw.nss.port = NSS_GPIO_Port;
  SX1278_hw.nss.pin = NSS_Pin;
  SX1278_hw.reset.port = Reset_GPIO_Port;
  SX1278_hw.reset.pin = Reset_Pin;
  SX1278_hw.spi = &hspi1;
  SX1278.hw = &SX1278_hw;
  
#if defined DEBUG || defined LORA_MASTER_DEBUG
  printf("Configuring LoRa module\r\n");
#endif
  SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_17DBM, SX1278_LORA_SF_8, 
               SX1278_LORA_BW_20_8KHZ, 10);
#if defined DEBUG || defined LORA_MASTER_DEBUG
  printf("Done configuring LoRa module\r\n");
#endif


#if defined DEBUG || defined RELEASE
  ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
#elif defined LORA_MASTER_DEBUG || defined LORA_MASTER_RELEASE
  ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
#endif
  
  /* Infinite loop */
  for(;;)
  {
#if defined DEBUG || defined RELEASE
    osDelay(1000);
    ret = SX1278_LoRaRxPacket(&SX1278);
    //printf("R: %d\r\n", ret);
    if (ret > 0) {
      SX1278_read(&SX1278, (uint8_t *) buffer, ret);
#ifdef DEBUG
      printf(" (%d): %s\r\n", ret, buffer);
#endif
    }
#endif
    
#if defined LORA_MASTER_DEBUG || defined LORA_MASTER_RELEASE
    message_length = sprintf(buffer, "Speed %d", message);
    ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
    
  #ifdef LORA_MASTER_DEBUG 
      printf("SX1278_LoRaEntryTx ret = (%d)\r\n", ret);
  #endif
    
    ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t *) buffer, message_length,
                                             2000);

  #ifdef LORA_MASTER_DEBUG 
      printf("SX1278_LoRaTxPacket ret = (%d)\r\n", ret);
  #endif    

    osDelay(5000);
    
    switch(message){
    case 0: 
      message += 1;
      break;
    case 1: 
      message += 1;
      break;
    case 2: 
      message = 0;
      break;
    }
#endif
}
}
/* USER CODE BEGIN Header_StartTaskFan */
/**
* @brief Function implementing the Fan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskFan */
void StartTaskFan(void const * argument)
{
  /* USER CODE BEGIN StartTaskFan */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartTaskFan */
}

/* USER CODE BEGIN Header_StartMcuWork */
/**
* @brief Function implementing the mcuWork thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMcuWork */
void StartMcuWork(void const * argument)
{
  /* USER CODE BEGIN StartMcuWork */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_RESET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_SET);
    osDelay(1000);
  }
  /* USER CODE END StartMcuWork */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
