/*
 * kalman_filter.h
 *
 *  Created on: Jul 12, 2024
 *      Author: Admin
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "kalman_filter.h"




typedef struct {
    float q_angle;
    float r_measure;
    float angle;
    float p[2][2];
} KalmanFilter;

void kalman_filter_init(KalmanFilter* kf) ;
float kalman_filter_get_angle(KalmanFilter* kf, float new_angle, float dt);

#endif
