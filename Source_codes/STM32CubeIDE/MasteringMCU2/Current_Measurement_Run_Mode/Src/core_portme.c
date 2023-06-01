/*
 File : core_portme.c
*/
/*
 Author : Shay Gal-On, EEMBC
 Legal : TODO!
*/
#include <stdio.h>
#include <stdlib.h>
#include "coremark.h"
#include "pwr_modes.h"

/* CODE & PRODUCT CONFIGURATION ----------------------------------------------*/
#include "stm32f4xx.h"

volatile ee_u32 Begin_Time = 0x00u, End_Time = 0x00u;

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;
/* Porting : Timing functions
 How to capture time and convert to seconds must be ported to whatever is supported by the platform.
 e.g. Read value from on board RTC, read value from cpu clock cycles performance counter etc.
 Sample implementation for standard time.h and windows.h definitions included.
*/
/* Define : TIMER_RES_DIVIDER
 Divider to trade off timer resolution and total time that can be measured.

 Use lower values to increase resolution, but make sure that overflow does not occur.
 If there are issues with the return value overflowing, increase this value.
 */
#define NSECS_PER_SEC CLOCKS_PER_SEC
#define CORETIMETYPE ee_u32

#define TIMER_RES_DIVIDER 1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC (NSECS_PER_SEC / TIMER_RES_DIVIDER)

/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/* Function : start_time
 This function will be called right before starting the timed portion of the benchmark.

 Implementation may be capturing a system timer (as implemented in the example code)
 or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void)
{
  Begin_Time = 0;
  start_time_val  = 0;

  /* Enable the TIM Counter */
  TIM5->CR1 |= TIM_CR1_CEN;
}
/* Function : stop_time
 This function will be called right after ending the timed portion of the benchmark.

 Implementation may be capturing a system timer (as implemented in the example code)
 or other system parameters - e.g. reading the current value of cpu cycles counter.
*/
void stop_time(void)
{
  stop_time_val = TIM5->CNT;
  /* Disable the TIM Counter */
  TIM5->CR1 &= ~TIM_CR1_CEN;

  End_Time = stop_time_val;
}
/* Function : get_time
 Return an abstract "ticks" number that signifies time on the system.

 Actual value returned may be cpu cycles, milliseconds or any other value,
 as long as it can be converted to seconds by <time_in_secs>.
 This methodology is taken to accomodate any hardware or simulated platform.
 The sample implementation returns millisecs by default,
 and the resolution is controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS get_time(void)
{
  CORE_TICKS elapsed = (CORE_TICKS)((stop_time_val - start_time_val));
  return elapsed;
}
/* Function : time_in_secs
 Convert the value returned by get_time to seconds.

 The <secs_ret> type is used to accomodate systems with no support for floating point.
 Default implementation implemented by the EE_TICKS_PER_SEC macro above.
*/
secs_ret time_in_secs(CORE_TICKS ticks)
{
  secs_ret retval = ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
  return retval;
}

ee_u32 default_num_contexts = 1;

/* Function : portable_init
 Target specific initialization code
 Test for some common mistakes.
*/
void portable_init(core_portable *p, int *argc, char *argv[])
{
  if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
  {
    ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
  }
  if (sizeof(ee_u32) != 4)
  {
    ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
  }
  p->portable_id = 1;
}

/* Function : portable_fini
 Target specific final code
*/
void portable_fini(core_portable *p)
{
  p->portable_id = 0;
}

/************************ STMicroelectronics add-on ***************************/
/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the
  *         SystemFrequency variable.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

}

/************************** STMicroelectronics add-on *************************/
