/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
