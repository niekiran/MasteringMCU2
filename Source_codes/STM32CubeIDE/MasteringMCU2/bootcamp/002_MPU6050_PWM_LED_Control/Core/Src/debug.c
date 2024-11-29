/*
 * debug.c
 *
 *  Created on: Jul 26, 2024
 *      Author: Admin
 */



#ifdef DEBUG
#include<stdint.h>
volatile float d_roll_angle;
volatile float d_dt;
volatile int16_t d_roll_angle_filt;
#endif

