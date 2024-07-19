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
#define HSE_DISCO 8

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

/************************  STANDBY mode function ******************************/
/**
  * @brief  This function configures the system to enter Standby mode with RTC
  *         clocked by LSI and with Backup SRAM ON for current consumption
  *         measurement purpose.
  *         STANDBY Mode with RTC clocked by LSI and BKPSRAM ON
  *         ===================================================
  *           - RTC Clocked by LSI
  *           - Backup SRAM ON
  *           - IWDG OFF
  *           - Automatic Wakeup using RTC clocked by LSI (after ~20s)
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StandbyBkpSramOnRtcON (void)
{
	 /* Allow access to the backup domain (RTC registers, RTC
	     backup data registers and backup SRAM) */
	  PWR_BackupAccessCmd(ENABLE);

	  /* LSI used as RTC source clock*/

	  /* Enable the LSI OSC */
	  RCC_LSICmd(ENABLE);

	  /* Wait till LSI is ready */
	  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	  {}

	  /* Select the RTC Clock Source */
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	  /* Enable the RTC Clock */
	  RCC_RTCCLKCmd(ENABLE);

	  /* Wait for RTC APB registers synchronisation */
	  RTC_WaitForSynchro();

	  /*-------------------- Backup SRAM -------------------*/

	  /* Enable BKPRAM Clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

	  /* Enable the Backup SRAM low power Regulator */
	  PWR_BackupRegulatorCmd(ENABLE);

	  /* Wait until the Backup SRAM low power Regulator is ready */
	  while (PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET)
	  {}
	  /*-----------------------------------------------------*/

	  /* RTC Wakeup Interrupt Generation: Clock Source: RTCCLK_Div16, Wakeup Time Base: ~20s
	      RTC Clock Source LSI ~32KHz

	      Wakeup Time Base = (16 / LSI) * WakeUpCounter
	   */

	  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
	  RTC_SetWakeUpCounter(WakeupCounter);

	  /* Enable the Wakeup Interrupt */
	  RTC_ITConfig(RTC_IT_WUT, ENABLE);

	  /* Enable Wakeup Counter */
	  RTC_WakeUpCmd(ENABLE);

	  /* Clear WakeUp (WUTF) pending flag */
	  RTC_ClearFlag(RTC_FLAG_WUTF);

	  /* Enable the wakeup pin */
	  PWR_WakeUpPinCmd(PWR_WakeUp_Pin1,ENABLE);

	  /* Clear standby flag */
	  PWR_ClearFlag(PWR_FLAG_SB);

	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Request to enter STANDBY mode*/
	  PWR_EnterSTANDBYMode();

}

/**
 * @brief  This function configures the system to enter Standby mode with Backup SRAM ON
 *         for current consumption measurement purpose.
 *         STANDBY Mode with BKPSRAM ON and RTC OFF
 *         ========================================
 *           - Backup SRAM ON RTC OFF
 *           - IWDG and LSI OFF
 *           - Wakeup using WakeUp Pin (PA.00)
 * @param  None
 * @retval None
 */
void PWR_StandbyBkpSramOnRtcOff (void)
{
	/* Allow access to the backup domain */
	  PWR_BackupAccessCmd(ENABLE);

	  /* Enable BKPRAM Clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

	  /* Enable the Backup SRAM low power Regulator */
	  PWR_BackupRegulatorCmd(ENABLE);

	  /* Wait until the Backup SRAM low power Regulator is ready */
	  while (PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET)
	  {}

	  /* Enable the wakeup pin */
	  PWR_WakeUpPinCmd(PWR_WakeUp_Pin1,ENABLE);

	  /* Clear standby flag */
	  PWR_ClearFlag(PWR_FLAG_SB);

	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Request to enter STANDBY mode*/
	  PWR_EnterSTANDBYMode();

}


/**
  * @brief  This function configures the system to enter Standby mode with RTC
  *         clocked by LSI for current consumption measurement purpose.
  *         STANDBY Mode with RTC clocked by LSI
  *         ====================================
  *           - RTC Clocked by LSI
  *           - IWDG OFF
  *           - Backup SRAM OFF
  *           - Automatic Wakeup using RTC clocked by LSI (after ~20s)
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StandbyBkpSramOffRtcOn (void)
{
	 /* Allow access to the backup domain (RTC registers, RTC
	  *     backup data registers and backup SRAM) */
	  PWR_BackupAccessCmd(ENABLE);

	  /* LSI used as RTC source clock*/
	  /* The RTC Clock may varies due to LSI frequency dispersion. */

	  /* Enable the LSI OSC */
	  RCC_LSICmd(ENABLE);

	  /* Wait till LSI is ready */
	  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	  {}

	  /* Select the RTC Clock Source */
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	  /* Enable the RTC Clock */
	  RCC_RTCCLKCmd(ENABLE);

	  /* Wait for RTC APB registers synchronisation */
	  RTC_WaitForSynchro();

	  /* RTC Wakeup Interrupt Generation: Clock Source: RTCCLK_Div16, Wakeup Time Base: ~20s
	     RTC Clock Source LSI ~32KHz

	     Wakeup Time Base = (16 / LSI) * WakeUpCounter
	  */
	  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
	  RTC_SetWakeUpCounter(WakeupCounter);

	  /* Enable the Wakeup Interrupt */
	  RTC_ITConfig(RTC_IT_WUT, ENABLE);

	  /* Clear WakeUp (WUTF) pending flag */
	  RTC_ClearFlag(RTC_FLAG_WUTF);

	  /* Enable Wakeup Counter */
	  RTC_WakeUpCmd(ENABLE);

	  /* Enable the wakeup pin */
	  PWR_WakeUpPinCmd(PWR_WakeUp_Pin1,ENABLE);

	  /* Clear standby flag */
	  PWR_ClearFlag(PWR_FLAG_SB);

	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Request to enter STANDBY mode */
	  PWR_EnterSTANDBYMode();

}

/**
  * @brief  This function configures the system to enter Standby mode for
  *         current consumption measurement purpose.
  *         STANDBY Mode
  *         ============
  *           - Backup SRAM and RTC OFF
  *           - IWDG and LSI OFF
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void PWR_StandbyBkpSramOffRtcOff (void)
{
	/* Clear standby flag */
	  PWR_ClearFlag(PWR_FLAG_SB);

	  /* Clear Wakeup flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

	  /* Enable WKUP pin 1 */
	  PWR_WakeUpPinCmd(PWR_WakeUp_Pin1,ENABLE);

	  /* Request to enter STANDBY mode*/
	  PWR_EnterSTANDBYMode();


}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

