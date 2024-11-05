/*
 * mpu6050.h
 *
 *  Created on: Jul 8, 2024
 *      Author: Admin
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f3xx_hal.h"

typedef enum {
	MPU6050_OK,
	MPU6050_ERR,
	MPU6050_REG_WRITE_FAIL,
	MPU6050_MEM_WRITE_FAIL,
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
	RAW_RDY_INT = 0x01,
	DMP_INT = 0X02,
	PLL_RDY_INT = 0x04,
	I2C_MST_INT = 0x08,
	FIFO_OFLOW_INT = 0x10,
	ZMOT_INT = 0x20,
	MOT_INT = 0x40,
	FF_INT = 0x80,
	ALL_INT = 0xFF
}mpu6050_interrupt_t;

typedef enum {
	INT_LEVEL_ACTIVE_HIGH = 0x00,
	INT_LEVEL_ACTIVE_LOW,
}mpu6050_interrupt_config_t;

typedef enum {
	MOT_ZRMOT = 0x01,
	MOT_ZPOS = 0x04,
	MOT_ZNEG = 0x08,
	MOT_YPOS = 0x10,
	MOT_YNEG = 0x20,
	MOT_XPOS = 0x40,
	MOT_XNEG = 0x80,
}mpu6050_motion_detection_status_t;

typedef enum {
	SLV0_FIFO = 0x01,
	SLV1_FIFO = 0x02,
	SLV2_FIFO = 0x04,
	ACCEL_FIFO = 0x08,
	ZG_FIFO = 0x10,
	YG_FIFO = 0x20,
	XG_FIFO = 0x40,
	TEMP_FIFO = 0x80,
	ALL_FIFO_SOURCE = 0xFF,
	NO_FIFO_SOURCE = 0x00
}mpu6050_fifo_source_t;

typedef struct {
	uint8_t gyro[8];
	uint8_t accel[8];
	uint8_t temp[2];
	uint8_t quat[12];
}mpu6050_dmp_packet_t;

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

mpu6050_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr);
mpu6050_status_t is_mpu6050_sleeping(I2C_HandleTypeDef *hi2c, uint8_t *is_sleeping);
mpu6050_status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr,\
		                   mpu6050_accel_data_t *accel_data);
mpu6050_accel_data_t mpu6050_accelerometer_calibration(const mpu6050_accel_data_t *error_offset, \
						mpu6050_accel_data_t *raw_data);
mpu6050_status_t mpu6050_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf \
);

mpu6050_status_t mpu6050_dmp_write_firmware(I2C_HandleTypeDef *hi2c, uint8_t *firmware_data, \
		uint32_t length, bool verify);

mpu6050_status_t mpu6050_enable_interrupts(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_t interrupts);
mpu6050_status_t mpu6050_disable_interrupts(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_t interrupts);
mpu6050_status_t mpu6050_dmp_disable(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_dmp_enable(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_fifo_disable(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_fifo_enable(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_fifo_reset(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_set_motion_detection_thresold(I2C_HandleTypeDef *hi2c, uint8_t threshold);
mpu6050_status_t mpu6050_interrupt_config(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_config_t level_config);
mpu6050_status_t mpu6050_fifo_disable_sources(I2C_HandleTypeDef *hi2c, mpu6050_fifo_source_t sources);
mpu6050_status_t mpu6050_fifo_enable_sources(I2C_HandleTypeDef *hi2c, mpu6050_fifo_source_t sources);
mpu6050_status_t mpu6050_dmp_program_start_address(I2C_HandleTypeDef *hi2c, int16_t addr);
mpu6050_status_t mpu6050_reset(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_set_memory_bank(I2C_HandleTypeDef *hi2c, uint8_t bank, \
		bool prefetch_enabled, bool user_bank);
mpu6050_status_t mpu6050_set_memory_start_address(I2C_HandleTypeDef *hi2c, uint8_t mem_start_addr);
mpu6050_status_t mpu6050_read_memory_byte(I2C_HandleTypeDef *hi2c, uint8_t* data);
mpu6050_status_t mpu6050_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data);
mpu6050_status_t mpu6050_read_otp_valid_bit(I2C_HandleTypeDef *hi2c, uint8_t* otp);
mpu6050_status_t mpu6050_set_full_scale_gyro_range(I2C_HandleTypeDef *hi2c, uint8_t range);
mpu6050_status_t mpu6050_set_external_frame_sync(I2C_HandleTypeDef *hi2c, uint8_t sync);
mpu6050_status_t mpu6050_set_sample_rate(I2C_HandleTypeDef *hi2c, int8_t rate);
mpu6050_status_t mpu6050_set_clock_source(I2C_HandleTypeDef *hi2c, uint8_t source);
mpu6050_status_t mpu6050_reset_i2c_master(I2C_HandleTypeDef *hi2c);
mpu6050_status_t mpu6050_i2c_master_mode_disable(I2C_HandleTypeDef *hi2c);
void mpu6050_set_slave_address(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t address);
void mpu6050_interrupt_handle(I2C_HandleTypeDef *hi2c);
//callbacks
void mpu6050_free_fall_detection_callback(void);
void mpu6050_motion_detection_callback(void);
void mpu6050_zero_motion_detection_callback(void);
void mpu6050_raw_data_ready_callback(void);
void mpu6050_dmp_callabck(void);


mpu6050_status_t mpu6050_get_dmp_packet(I2C_HandleTypeDef *hi2c, mpu6050_dmp_packet_t *packet);

//MPU6050 reg addresses
#define MPU6050_REG_WHOAMI 		(uint8_t)117
#define MPU6050_REG_PWMGMT_1 	(uint8_t)0x6B
#define MPU6050_REG_ACCEL_START (uint8_t)59
#define MPU6050_REG_CONFIG      (uint8_t)26
#define MPU6050_REG_USER_CTRL	(uint8_t)0x6A
#define MPU6050_REG_BANK_SEL    (uint8_t)0x6D
#define MPU6050_REG_MEM_START_ADDR   0x6E
#define MPU6050_REG_MEM_R_W          0x6F
#define MPU6050_REG_INT_STATUS     0x3A
#define MPU6050_REG_INT_EN         0x38
#define MPU6050_REG_MOT_THR 0x1F
#define MPU6050_REG_MOT_DUR 0x20
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_FIFO_SOURCE_EN 0x23
#define MPU6050_REG_DMP_PROG_ADDR 0x70
#define MPU6050_REG_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_REG_FIFO_COUNT_H 0x72
#define MPU6050_REG_FIFO_COUNT_L 0x73
#define MPU6050_REG_FIFO_R_W 0x74

#endif /* SRC_MPU6050_H_ */
