/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include "main_app.h"

extern CAN_HandleTypeDef hcan1;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void CAN1_TX_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

}

void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

}

void CAN1_RX1_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

}

void CAN1_SCE_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

}







