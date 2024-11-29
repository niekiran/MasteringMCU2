/*
 * mpu6050.h
 *
 *  Created on: Jul 8, 2024
 *      Author: Admin
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_

#include <stdint.h>
#include "stm32f3xx_hal.h"

typedef enum {
	MPU6050_OK,
	MPU6050_ERR
}mpu6050_status_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}mpu6050_accel_data_t;

typedef enum {
	DLPF_CFG_260HZ = 0,
	DLPF_CFG_184HZ = 1,
	DLPF_CFG_94HZ = 2,
	DLPF_CFG_44HZ = 3,
	DLPF_CFG_21HZ = 4,
	DLPF_CFG_10HZ = 5,
	DLPF_CFG_5HZ = 6,

}mpu6050_dlpf_config_t;


typedef enum {
	INT_LEVEL_ACTIVE_HIGH = 0x00,
	INT_LEVEL_ACTIVE_LOW,
}mpu6050_interrupt_config_t;


typedef enum {
	RAW_RDY_INT = 0x01,
	I2C_MST_INT = 0x08,
	FIFO_OFLOW_INT = 0x10,
	MOT_INT = 0x40,
	ALL_INT = 0xFF
}mpu6050_interrupt_t;

mpu6050_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr);
mpu6050_status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr,\
		                   mpu6050_accel_data_t *accel_data);
mpu6050_accel_data_t mpu6050_accelerometer_calibration(const mpu6050_accel_data_t *error_offset, \
						mpu6050_accel_data_t *raw_data);
mpu6050_status_t mpu6050_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf \
);
mpu6050_status_t mpu6050_enable_interrupt(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_t interrupt);
mpu6050_status_t mpu6050_disable_interrupt(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_t interrupt);
mpu6050_status_t mpu6050_interrupt_config(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_config_t level_config);
void mpu6050_interrupt_handle(I2C_HandleTypeDef *hi2c);


//callbacks
void mpu6050_motion_detection_callback(void);
void mpu6050_raw_data_ready_callback(void);


//MPU6050 reg addresses
#define MPU6050_REG_WHOAMI 		(uint8_t)117
#define MPU6050_REG_PWMGMT_1 	(uint8_t)107
#define MPU6050_REG_ACCEL_START (uint8_t)59
#define MPU6050_REG_CONFIG      (uint8_t)26
#define MPU6050_REG_INT_STATUS     		 0x3A
#define MPU6050_REG_INT_EN         		 0x38
#define MPU6050_REG_MOT_THR 			 0x1F
#define MPU6050_REG_MOT_DUR 			 0x20
#define MPU6050_REG_INT_PIN_CFG      	 0x37

#endif /* SRC_MPU6050_H_ */
