/**
  ******************************************************************************
  * @file    STM32F4xx_Current_Consumption_Measuring/src/pwr_modes.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   Power modes functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

/* CODE & PRODUCT CONFIGURATION ----------------------------------------------*/
#include "stm32f4xx.h"
#include "pwr_modes.h"
#include "main.h"

/** @addtogroup STM32F4xx current consumption
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WakeupCounter 0xA000 /*specifies the WakeUp counter*/
#define HSE_OF_NUCLEO_F446RE 8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
GPIO_InitTypeDef GPIO_InitStructure;


void DisableAllAHBxAPBxPeriClocks(void)
{
	/*Disabling clocks of all the peripherals in AHB1,2,3 and APB1,2 domain */
	//Clearing off all bits in the below registers to disable the clocks. for more info refer RM , RCC Register Section
	  RCC->AHB1ENR &= ~(0xFFFFFFFF);
	  RCC->AHB2ENR &= ~(0xFFFFFFFF);
	  RCC->AHB3ENR &= ~(0xFFFFFFFF);
	  RCC->APB1ENR &= ~(0xFFFFFFFF);
	  RCC->APB2ENR &= ~(0xFFFFFFFF);
}

void EnableAllAHBxAPBxPeriClocks(void)
{
	/*Enabling clocks of all the peripherals in AHB1,2,3 and APB1,2 domain */
	RCC->AHB1ENR |= 0x606410FF;
	RCC->AHB2ENR |= ( (1 << 0) | (1 << 7));
	RCC->AHB3ENR |= 0x00000003;
	RCC->APB1ENR |= 0x3FFFC9FF;
	RCC->APB2ENR |= 0x00C77F66;
}

/*****************  STOP mode function ****************************************/

/**
  * @brief  This function configures the system to enter Stop mode
  *         for current consumption measurement purpose.
  *         ===========================================
  *           - Regulator in Main mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in Stop mode
  *           - Wakeup using WakeUp Pin (PC13)
  * @param  None
  * @retval None
  */
void PWR_StopMainRegFlashStop (void)
{
	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Enable Button interrupt(our wakeup source) */
	 ButtonPinInt_configuration();

	  /* Disable FLASH Deep Power Down Mode by clearing FPDS bit*/
	  PWR_FlashPowerDownCmd(DISABLE);

	  /* Enter Stop Mode */
	  PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);

}

/**
  * @brief  This function configures the system to enter Stop mode
  *         for current consumption measurement purpose.
  *         ===========================================
  *           - Regulator in Main mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in deep power down mode
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StopMainRegFlashPwrDown (void)
{
	 /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);


	  /* Enable the wakeup pin */
	 ButtonPinInt_configuration();

	  /* FLASH Deep Power Down Mode enabled */
	  PWR_FlashPowerDownCmd(ENABLE);

	  /* Enter Stop Mode */
	  PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
}

/**
  * @brief  This function configures the system to enter Stop mode
  *         for current consumption measurement purpose.
  *         ===========================================
  *           - Regulator in Low Power mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in STOP mode
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StopLowPwrRegFlashStop (void)
{
	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Enable the wakeup pin */
	  ButtonPinInt_configuration();

	  /* Disable FLASH Deep Power Down Mode by clearing FPDS bit*/
	  PWR_FlashPowerDownCmd(DISABLE);

	  /* Enter Stop Mode */
	  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

}

/**
  * @brief  This function configures the system to enter Stop mode
  *         for current consumption measurement purpose.
  *         ===========================================
  *           - Regulator in Low Power mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in deep power down mode
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StopLowPwrRegFlashPwrDown (void)
{
	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Enable button interrupt pin */
	  ButtonPinInt_configuration();

	  /* FLASH Deep Power Down Mode enabled */
	  PWR_FlashPowerDownCmd(ENABLE);

	  /* Enter Stop Mode Reg LP*/
	  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

}

/**
  * @brief  This function configures the system to enter Stop mode
  *         for current consumption measurement purpose.
  *         ===========================================
  *           - Regulator in Main & under drive mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in deep power down mode
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StopMainRegUnderDriveFlashPwrDown (void)
{

	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Enable the wakeup pin */
	  ButtonPinInt_configuration();

	  /*Enables the Under-Drive mode. */
	  PWR_UnderDriveCmd(ENABLE);

	  /* Enter Stop Mode  low voltage*/
	  PWR_EnterUnderDriveSTOPMode(PWR_MainRegulator_UnderDrive_ON , PWR_STOPEntry_WFI);


}

/**
  * @brief  This function configures the system to enter Stop mode
  *         for current consumption measurement purpose.
  *         ===========================================
  *           - Regulator in Low Power & under drive mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in deep power down mode
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StopLowPwrRegUnderDriveFlashPwrDown (void)
{

	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Enable the wakeup pin */
	  ButtonPinInt_configuration();

	  /*Enables the Under-Drive mode. */
	  PWR_UnderDriveCmd(ENABLE);

	  /* Enter Stop Mode  low voltage*/
	  PWR_EnterUnderDriveSTOPMode(PWR_LowPowerRegulator_UnderDrive_ON, PWR_STOPEntry_WFI);


}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

