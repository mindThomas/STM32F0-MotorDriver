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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"

#ifdef USE_FREERTOS
#include "cmsis_os.h"
#endif
 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t uwTickHighRes;
uint32_t frequency;
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
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  
  /* Enable TIM16 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16); //__HAL_RCC_TIM16_CLK_ENABLE();
  
  /* Compute TIM16 clock */
  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  uwTimclock = clocks.PCLK1_Frequency; //HAL_RCC_GetPCLK1Freq();

  frequency = 10000; // 10 kHz
   
  /* Compute the prescaler value to have TIM16 counter clock equal to 10 kHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / frequency) - 1);
  
  /* Initialize TIM16 peripheral as follow:
  + Period = [(TIM16CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  // Modified by Thomas: Configure timer to overflow every second, thus firing an interrupt @ 1 Hz rate
  LL_TIM_SetPrescaler(TIM16, uwPrescalerValue);
  LL_TIM_SetAutoReload(TIM16, (frequency / 1) - 1);

  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM16);

  /*Configure the TIM16 IRQ priority */
  NVIC_SetPriority(TIM16_IRQn, TickPriority);

  /* Enable the TIM16 global Interrupt */
  NVIC_EnableIRQ(TIM16_IRQn);

  /* Enable counter */
  LL_TIM_EnableCounter(TIM16);

  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM16);

  /* Return function status */
  return HAL_ERROR;
}

uint32_t HAL_GetTickTimerValue(void)
{
	return (uint32_t)LL_TIM_GetCounter(TIM16);
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
	LL_TIM_DisableIT_UPDATE(TIM16); // __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);
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
  LL_TIM_EnableIT_UPDATE(TIM16);
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
	/* Check whether update interrupt is pending */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == 1)
	{
		/* Clear the update interrupt flag*/
		LL_TIM_ClearFlag_UPDATE(TIM16);

		// Increase the timer tick
		HAL_IncTick();
	}
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
	uwTickHighRes += 10000; // timer update rate configured 1 Hz but with timer count frequency running at 10 kHz
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

#ifdef USE_FREERTOS
void HAL_Delay(uint32_t Delay)
{
	osDelay(Delay);
}
#else
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
#endif

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
  return (HAL_GetHighResTick() / 10); // divide by 10 since we have configured HAL timer to run at 10 kHz in stm32h7xx_hal_timebase_tim.c
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
