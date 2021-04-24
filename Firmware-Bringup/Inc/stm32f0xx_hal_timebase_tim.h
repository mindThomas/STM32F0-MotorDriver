#ifndef __STM32F0xx_HAL_TIMEBASE_TIM_H
#define __STM32F0xx_HAL_TIMEBASE_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

uint32_t HAL_GetTickTimerValue(void);
uint32_t HAL_GetHighResTick(void);
void HAL_DelayHighRes(uint32_t Delay);

uint32_t HAL_tic();
float HAL_toc(uint32_t timerPrev);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_HAL_TIMEBASE_TIM_H */
