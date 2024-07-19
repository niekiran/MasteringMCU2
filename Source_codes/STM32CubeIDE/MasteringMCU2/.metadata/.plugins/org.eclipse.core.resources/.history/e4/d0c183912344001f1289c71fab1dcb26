/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include "main_app.h"

extern RTC_HandleTypeDef hrtc;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();

}


void EXTI15_10_IRQHandler(void)
{

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}


void RTC_Alarm_IRQHandler(void)
{
	HAL_RTC_AlarmIRQHandler(&hrtc);
}

