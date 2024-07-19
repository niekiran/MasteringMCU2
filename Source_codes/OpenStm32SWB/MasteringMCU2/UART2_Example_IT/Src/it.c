/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include <main.h>

extern UART_HandleTypeDef huart2;

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt.
  */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}
