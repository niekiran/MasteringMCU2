/*
 * app_callback.c
 *
 *  Created on: Jul 5, 2024
 *      Author: Admin
 */

#include "main.h"
#include "mpu6050.h"
#include "cmsis_os.h"
#include "task.h"
#include "debug.h"

extern  uint32_t g_counter;
extern volatile uint32_t g_channel_2_state;
extern volatile uint32_t g_channel_3_state;
extern I2C_HandleTypeDef hi2c1;
extern osThreadId_t SensorReadTask_Handle;

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


 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if (GPIO_Pin == MPU6050_INT_Pin) {
		 mpu6050_interrupt_handle(&hi2c1);
	 }
}



 //this is the ISR function
void mpu6050_raw_data_ready_callback(void) {
	BaseType_t xHigherPriorityTaskWoken;

	uint32_t dt;
	static uint32_t previous_tick = 0;

	//uint32_t current_tick = osKernelGetTickCount();
	uint32_t current_tick = DWT->CYCCNT;
	if (previous_tick == 0) {
		 //previous_tick = osKernelGetTickCount();
		 previous_tick = DWT->CYCCNT;
	}

	dt = (current_tick - previous_tick);
	previous_tick = current_tick;
	float dt_new = dt / (SystemCoreClock * 1.0f);
	float dt_debug = dt_new * 1000.0f;
	DEBUG_DT(dt_debug);

	xHigherPriorityTaskWoken = pdFALSE;


	if(SensorReadTask_Handle) {

	xTaskNotifyFromISR( (TaskHandle_t)SensorReadTask_Handle,
	                               dt,
								   eSetValueWithOverwrite,
	                               &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
