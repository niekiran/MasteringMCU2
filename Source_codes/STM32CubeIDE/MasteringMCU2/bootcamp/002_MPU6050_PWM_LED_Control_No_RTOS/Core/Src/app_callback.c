/*
 * app_callback.c
 *
 *  Created on: Jul 5, 2024
 *      Author: Admin
 */

#include "main.h"

extern  uint32_t g_counter;
extern volatile uint32_t g_channel_2_state;
extern volatile uint32_t g_channel_3_state;


#if 0
//called during update event of the counter
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  g_counter = 0;
  g_channel_2_state = 80;
  g_channel_3_state = 40;

}
#endif

 /**
   * @brief  PWM Pulse finished callback in non-blocking mode
   * @param  htim TIM handle
   * @retval None
   */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
 {
   /* Prevent unused argument(s) compilation warning */
     //UNUSED(htim);
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		g_channel_2_state = 0;
	}

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		g_channel_3_state = 0;
	}


 }


/**
  * @brief  Timer error callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
 void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);


}
