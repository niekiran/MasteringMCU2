/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include "main.h"

extern TIM_HandleTypeDef htimer6;

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/**
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer6);
}
