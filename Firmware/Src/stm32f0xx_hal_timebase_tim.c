/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_hal_timebase_TIM.c 
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"
 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim16; 
static __IO uint32_t uwTickHighRes;
static uint32_t frequency;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM16 as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIM16 IRQ priority */
  HAL_NVIC_SetPriority(TIM16_IRQn, TickPriority ,0); 
  
  /* Enable the TIM16 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM16_IRQn); 
  
  /* Enable TIM16 clock */
  __HAL_RCC_TIM16_CLK_ENABLE();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute TIM16 clock */
  uwTimclock = HAL_RCC_GetPCLK1Freq();
   
  frequency = 100000; // 100 kHz

  /* Compute the prescaler value to have TIM16 counter clock equal to 100 kHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / frequency) - 1);
  
  /* Initialize TIM16 */
  htim16.Instance = TIM16;
  
  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM16CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  // Modified by Thomas: Configure timer to overflow every second, thus firing an interrupt @ 10 Hz rate
  htim16.Init.Period = (frequency / 10) - 1;
  htim16.Init.Prescaler = uwPrescalerValue;
  htim16.Init.ClockDivision = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim16) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim16);
  }
  
  /* Return function status */
  return HAL_ERROR;
}

uint32_t HAL_GetTickTimerValue(void)
{
	return (uint32_t)__HAL_TIM_GET_COUNTER(&htim16);
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM16 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);                                                  
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM16 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
}

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
void HAL_IncTick(void)
{
	uwTickHighRes += 10000; // timer update rate configured 10 Hz but with timer count frequency running at 100 kHz
}

uint32_t HAL_GetHighResTick(void)
{
	return (uwTickHighRes + HAL_GetTickTimerValue());
}

uint32_t HAL_tic()
{
	return HAL_GetHighResTick();
}

float HAL_toc(uint32_t timerPrev)
{
	uint32_t timerDelta;
	uint32_t timerNow = HAL_GetHighResTick();
	if (timerNow > timerPrev)
		timerDelta = timerNow - timerPrev;
	else
		timerDelta = ((uint32_t)0xFFFFFFFF - timerPrev) + timerNow;

	float microsTime = (float)timerDelta / frequency;
	return microsTime;
}

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /*
  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  */

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}

void HAL_DelayHighRes(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetHighResTick();
  uint32_t wait = Delay;

  /*
  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  */

  while ((HAL_GetHighResTick() - tickstart) < wait)
  {
  }
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return (HAL_GetHighResTick() / 100); // divide by 100 since we have configured HAL timer to run at 100 kHz in stm32h7xx_hal_timebase_tim.c
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
