/*
 * main.h
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f4xx_hal.h"

#define SYS_CLOCK_FREQ_50_MHZ 50
#define SYS_CLOCK_FREQ_84_MHZ 84
#define SYS_CLOCK_FREQ_120_MHZ 120


#define LED1_PORT GPIOC
#define LED2_PORT GPIOC
#define LED3_PORT GPIOB
#define LED4_PORT GPIOC

#define LED1_PIN_NO GPIO_PIN_9
#define LED2_PIN_NO GPIO_PIN_8
#define LED3_PIN_NO GPIO_PIN_8
#define LED4_PIN_NO GPIO_PIN_6

#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
//#define RTC_CLOCK_SOURCE_LSE
#define RTC_CLOCK_SOURCE_LSI

#define TRUE 1
#define FALSE 0

#endif /* MAIN_H_ */
