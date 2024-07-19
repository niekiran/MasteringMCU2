/**
  ******************************************************************************
  * @file    STM32F4xx_Current_Consumption_Measuring/inc/test_config.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   This file contains the headers of test configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_pwr_modes_H
#define __STM32F4xx_pwr_modes_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*************************** ART accelerator configuration*********************/

/* Uncomment the macros to enable ART with or without prefetch buffer */

/*Enable ART*/
 // #define ART_Enable

/*Enbale prefetch when ART is enabled */
 // #define  Prefetch_Enable

/********************* Power supply selection *********************************/

/* Uncomment the macros to select 3,3 V or 1,8 V as power supply */

 #define VDD3_3

 /************************* Power sub-modes selection **************************/

/* Uncomment macro(s) to select sub-mode(s) to be measured for each power mode*/


/*--------------------- Standby sub-mode selection ---------------------------*/

#define StandbyBkpSramOnRtcON
#define StandbyBkpSramOffRtcOn
#define StandbyBkpSramOnRtcOff
#define StandbyBkpSramOffRtcOff

 /**************************** STANDBY mode  ***********************************/

/* Standby mode, Backup SRAM ON, RTC ON */
void PWR_StandbyBkpSramOnRtcON (void);

/* Standby mode, Backup SRAM ON, RTC OFF */
void PWR_StandbyBkpSramOnRtcOff (void);

/* Standby mode, Backup SRAM OFF, RTC ON */
void PWR_StandbyBkpSramOffRtcOn (void);

/* Standby mode, Backup SRAM OFF, RTC OFF */
void PWR_StandbyBkpSramOffRtcOff (void);


#define ALL_GPIOs           (RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_GPIOC | \
                            RCC_AHB1Periph_GPIOD| RCC_AHB1Periph_GPIOE| RCC_AHB1Periph_GPIOF | \
                            RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_GPIOH)


#endif /* __STM32F4xx_pwr_modes_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
