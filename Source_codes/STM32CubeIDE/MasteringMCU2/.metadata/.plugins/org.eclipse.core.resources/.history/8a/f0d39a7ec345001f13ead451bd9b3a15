
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pwr_modes.h"


__IO uint32_t uwCounter = 0x00;
__IO uint8_t UserButtonStatus = RESET;

static void Mode_Exit (void);
static void LedsConfig (void);

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

	/* Enable PWR APB1 Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Configure CPIOs as analog inputs */
	GPIO_AnalogConfig ();

	/*Configure Leds GPIOs */
	LedsConfig();

	/* Configure Wakeup pin  */
	ButtonPinInt_configuration();

	/* Execute defined Standby modes */
	Measure_Standby ();

	/* Infinite loop */
	while (1)
	{}


}

/**
* @brief  Configure the Standby modes exection routines.
* @param  None
* @retval None
*/
void Measure_Standby (void)
{

#if defined StandbyBkpSramOnRtcON

  /* Wait User puch buttom to enter Standby mode */
  WaitUser_PushBottom ();

  /* Configure CPIOs as analog inputs */
  GPIO_AnalogConfig ();

  PWR_StandbyBkpSramOnRtcON ();

#endif /* StandbyBkpSramOnRtcON */

#if defined StandbyBkpSramOffRtcOn

  WaitUser_PushBottom ();

  /* Configure CPIOs as analog inputs */
  GPIO_AnalogConfig ();

  PWR_StandbyBkpSramOffRtcOn ();

#endif /* StandbyBkpSramOffRtcOn */

#if defined StandbyBkpSramOnRtcOff

  /* Wait User puch buttom to enter Standby mode */
  WaitUser_PushBottom ();

  /* Configure CPIOs as analog inputs */
  GPIO_AnalogConfig ();

  PWR_StandbyBkpSramOnRtcOff ();

#endif /* StandbyBkpSramOnRtcOff */

#if defined StandbyBkpSramOffRtcOff

  WaitUser_PushBottom ();

  /* Configure CPIOs as analog inputs */
  GPIO_AnalogConfig ();

  PWR_StandbyBkpSramOffRtcOff ();

#endif /* StandbyBkpSramOffRtcOff */

}

/**
* @brief  Configure All GPIO as analog input.
* @param  None
* @retval None
*/
void GPIO_AnalogConfig (void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;

	  /* Configure GPIOs as Analog input to reduce current consumption*/
	  /* Enable GPIOs clock */
	  RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | \
	                          RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | \
	                          RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOH , ENABLE);

	  RCC_AHB1PeriphClockCmd ((RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG),ENABLE );

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
	  GPIO_Init(GPIOE, &GPIO_InitStructure);
	  GPIO_Init(GPIOH, &GPIO_InitStructure);
	  GPIO_Init(GPIOF, &GPIO_InitStructure);
	  GPIO_Init(GPIOG, &GPIO_InitStructure);
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Disable GPIOs clock */
	  RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | \
	                          RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | \
	                          RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOH , DISABLE);

	  RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, DISABLE);
}


/**
* @brief  Configure LED gpio of NUCLEO-F446RE GPIOA.5
* @param  None
* @retval None
*/
static void LedsConfig (void)
{
/* Modification of code required if your board is not NUCLEO-F446RE*/
	GPIO_InitTypeDef  GPIO_InitStructure;

	/*Configure GPIO structure */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/* Enable the GPIO_LED Port A Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

	/* Configure 5th pin which drivers the LED*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/**
* @brief  Configure the PC13 pin as interrupt wakeup source over EXTI line
* @param  None
* @retval None
* @Note :  Modification of code required if your board is not NUCLEO-F446RE
*/
void ButtonPinInt_configuration(void)
{
	  NVIC_InitTypeDef NVIC_InitStructure;
#if 1
	  GPIO_InitTypeDef GPIO_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStructure;


	  /* Enable GPIOA clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PA0 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Connect EXTI Line0 to PA0 pin */
      SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);


	  /* Configure EXTI Line0 */
	  EXTI_DeInit();
	  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);



	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
#endif

#if 0
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	EXTI_DeInit();
	GPIOC->MODER &= ~(0X3 << 26);

	EXTI->IMR |= ( 1 << 13);



	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG->EXTICR[3] &= ~(0xf  << 4 );
	SYSCFG->EXTICR[3] |= (0x2  << 4 );
	EXTI->RTSR |= ( 1 << 13);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
}

//Just waits until user presses on board push button
void WaitUser_PushBottom (void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	while(UserButtonStatus != SET); //wait here
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	UserButtonStatus = RESET;
}

/**
* @brief  Configure the exit routine from power mode.
* @param  None
* @retval None
*/
static void Mode_Exit (void)
{
	/* Clock init configuration */
	  RCC_DeInit();

	  /* Disable HSE */
	  RCC_HSEConfig(RCC_HSE_OFF);

	  /* Enable HSI */
	  RCC_HSICmd(ENABLE);

	  /* Wait till HSI is ready */
	  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
	  {}

	  /* Select HSI as system clock source */
	  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

	  /* Wait till HSI is used as system clock source */
	  while (RCC_GetSYSCLKSource() != 0x00)
	  {}

	  /* Enable PWR APB1 Clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	  /*Configure Leds GPIOs */
	  LedsConfig();

	  /* Configure Wakeup pin  */
	  ButtonPinInt_configuration();
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{


}

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 UserButtonStatus = SET;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
