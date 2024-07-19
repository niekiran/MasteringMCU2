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

/*************************** ART accelerator configuration*********************/

/* Uncomment the macros to enable ART with or without prefetch buffer */

/*Enable ART*/
 // #define ART_Enable

/*Enbale prefetch when ART is enabled */
 // #define  Prefetch_Enable


/********************* Power supply selection *********************************/

 #define VDD3_3

/************************* Power sub-modes selection **************************/

/* Uncomment macro(s) to select sub-mode(s) to be measured for each power mode*/

/*----------------------- Sleep sub-modes selection  ------------------------*/
#define SleepPeriphDisabled180Mhz
//#define SleepPeriphEnabled180Mhz
//#define SleepPeriphDisabled60Mhz
//#define SleepPeriphEnabled60Mhz


 /**************************** SLEEP mode  ************************************/

 /* Sleep mode, 180 MHz O.Drive_scale1-ALL_Periph_disabled */
void PWR_SleepPeriphDisabled180Mhz(void);

/* Sleep mode, 180 MHz O.Drive_scale1-ALL_Periph_enabled */
void PWR_SleepPeriphEnabled180Mhz(void);

/* Sleep mode, 60 MHz_ALL_Periph_Disabled */
void PWR_SleepPeriphDisabled60Mhz(void);

/* Sleep mode, 60 MHz_ALL_Periph_Enabled */
void PWR_SleepPeriphEnabled60Mhz(void);



#define ALL_GPIOs           (RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_GPIOC | \
                            RCC_AHB1Periph_GPIOD| RCC_AHB1Periph_GPIOE| RCC_AHB1Periph_GPIOF | \
                            RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_GPIOH)


#endif /* __STM32F4xx_pwr_modes_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
