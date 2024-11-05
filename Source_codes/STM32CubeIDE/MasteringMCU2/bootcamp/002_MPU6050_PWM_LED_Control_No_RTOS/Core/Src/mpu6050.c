/*
 * mpu6050.c
 *
 *  Created on: Jul 8, 2024
 *      Author: Admin
 */


#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

#define BIT_IS_SET 1
#define BIT_IS_REST 0

#define CHECK_AND_HANDLE_ERROR(func_call) \
    do { \
        mpu6050_status_t status = (func_call); \
        if (status != MPU6050_OK) { \
            return status; \
        } \
    } while(0)


volatile uint8_t verify_buf[256];

static uint8_t mpu6050_i2c_addr;
#define I2C_TIMEOUT 500UL

static mpu6050_status_t set_memory_bank(I2C_HandleTypeDef *hi2c, uint8_t bank, \
		bool prefetch_enabled, bool user_bank);
static mpu6050_status_t set_memory_start_address(I2C_HandleTypeDef *hi2c, uint8_t mem_start_addr);


mpu6050_status_t mpu6050_read_byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data) {
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c, mpu6050_i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT , data, 1, I2C_TIMEOUT);

	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERR;

}

mpu6050_status_t mpu6050_reset(I2C_HandleTypeDef *hi2c) {

	uint8_t pwm_mgmt_1 = 0;
	//if (mpu6050_read_byte(hi2c, MPU6050_REG_PWMGMT_1, &pwm_mgmt_1) != MPU6050_OK) {
		//return MPU6050_ERR;
	//}

	pwm_mgmt_1 |= (1 << 7);
	return mpu6050_write_byte(hi2c, MPU6050_REG_PWMGMT_1, pwm_mgmt_1);

}

mpu6050_status_t mpu6050_read(I2C_HandleTypeDef *hi2c, uint8_t reg_base_addr, uint8_t *buffer, uint32_t nbytes) {
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c, mpu6050_i2c_addr << 1, reg_base_addr, I2C_MEMADD_SIZE_8BIT , buffer, nbytes, HAL_MAX_DELAY);

	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERR;

}


mpu6050_status_t mpu6050_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data) {
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Write(hi2c, mpu6050_i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT , &data, 1, I2C_TIMEOUT);

	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERR;

}

mpu6050_status_t mpu6050_write(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data, uint32_t len) {
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Write(hi2c, mpu6050_i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT , data, len, I2C_TIMEOUT);

	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERR;


}


void mpu6050_dmp_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr) {

		//PWR_MGMT_1: reset with 100ms delay
	// full SIGNAL_PATH_RESET: with another 100ms delay

}


mpu6050_status_t mpu6050_fifo_enable(I2C_HandleTypeDef *hi2c) {

	uint8_t user_ctrl;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	user_ctrl |= (1 << 6);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);

}

mpu6050_status_t mpu6050_fifo_reset(I2C_HandleTypeDef *hi2c) {

	uint8_t user_ctrl;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	user_ctrl |= (1 << 2);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);

}

mpu6050_status_t mpu6050_fifo_disable(I2C_HandleTypeDef *hi2c) {

	uint8_t user_ctrl;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	user_ctrl &= ~(1 << 6);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);
}


mpu6050_status_t mpu6050_fifo_enable_sources(I2C_HandleTypeDef *hi2c, mpu6050_fifo_source_t sources) {
	uint8_t current_fifo_sources = 0;

	if (sources != ALL_FIFO_SOURCE){
		mpu6050_read_byte(hi2c, MPU6050_REG_FIFO_SOURCE_EN, &current_fifo_sources);
	}

	current_fifo_sources |= sources;
	return mpu6050_write_byte(hi2c, MPU6050_REG_FIFO_SOURCE_EN, current_fifo_sources);
}

mpu6050_status_t mpu6050_fifo_disable_sources(I2C_HandleTypeDef *hi2c, mpu6050_fifo_source_t sources) {
	uint8_t current_fifo_sources = 0;

	if (sources != ALL_FIFO_SOURCE){
		mpu6050_read_byte(hi2c, MPU6050_REG_FIFO_SOURCE_EN, &current_fifo_sources);
	}

	current_fifo_sources &= (~sources);
	return mpu6050_write_byte(hi2c, MPU6050_REG_FIFO_SOURCE_EN, current_fifo_sources);
}


mpu6050_status_t mpu6050_set_motion_detection_threshold_and_duration(I2C_HandleTypeDef *hi2c, uint8_t threshold, uint8_t duration) {
	mpu6050_write_byte(hi2c, MPU6050_REG_MOT_THR, threshold);
	return mpu6050_write_byte(hi2c, MPU6050_REG_MOT_DUR, duration);
}

mpu6050_status_t mpu6050_dmp_program_start_address(I2C_HandleTypeDef *hi2c, int16_t addr) {
	uint8_t buf[2];
	buf[0] = (addr >> 8) & 0xff;
	buf[1] = (addr & 0xff);
	return mpu6050_write(hi2c, MPU6050_REG_DMP_PROG_ADDR, buf, 2);
}

mpu6050_status_t mpu6050_dmp_disable(I2C_HandleTypeDef *hi2c) {

	uint8_t user_ctrl;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	user_ctrl &= ~(1 << 7);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);

#if 0
	HAL_Delay(5);

	user_ctrl = 0;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}


	if ((user_ctrl & (1 << 7))) {
		return MPU6050_REG_WRITE_FAIL;
	}

	return MPU6050_OK;
#endif

}

mpu6050_status_t mpu6050_interrupt_config(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_config_t level_config) {

	uint8_t int_cfg = 0;
	int_cfg |= (level_config << 7);
	return mpu6050_write_byte(hi2c, MPU6050_REG_INT_PIN_CFG, int_cfg);
}


mpu6050_status_t mpu6050_disable_interrupts(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_t interrupts) {
	uint8_t current_int_settings = 0;
	if (interrupts != (uint8_t)ALL_INT) {
		mpu6050_read_byte(hi2c, MPU6050_REG_INT_EN, &current_int_settings);
	}

	current_int_settings &= ~interrupts;
	return mpu6050_write_byte(hi2c, MPU6050_REG_INT_EN, current_int_settings);
}


mpu6050_status_t mpu6050_enable_interrupts(I2C_HandleTypeDef *hi2c, mpu6050_interrupt_t interrupts) {
	uint8_t current_int_settings = 0;
	mpu6050_read_byte(hi2c, MPU6050_REG_INT_EN, &current_int_settings);
	current_int_settings |= (uint8_t)interrupts;
	return mpu6050_write_byte(hi2c, MPU6050_REG_INT_EN, current_int_settings);
}

mpu6050_status_t mpu6050_dmp_enable(I2C_HandleTypeDef *hi2c) {

	uint8_t user_ctrl;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	user_ctrl |= (1 << 7);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);

#if 0
	user_ctrl |= (1 << 7);
	if (mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	HAL_Delay(5);

	user_ctrl = 0;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl) != MPU6050_OK) {
		return MPU6050_ERR;
	}


	if (!(user_ctrl & (1 << 7))) {
		return MPU6050_REG_WRITE_FAIL;
	}

	return MPU6050_OK;
#endif

}


mpu6050_status_t mpu6050_dmp_write_firmware(I2C_HandleTypeDef *hi2c, uint8_t *firmware_data, uint32_t length, bool verify) {

	uint32_t remaining_bytes = 0;
	uint32_t bytes_to_write = 0;
	uint8_t bank = 0;
	uint8_t *write_ptr  = firmware_data;

	//CHECK_AND_HANDLE_ERROR(set_memory_bank(hi2c, 0, false, false));
	//CHECK_AND_HANDLE_ERROR(set_memory_start_address(hi2c, 0));

	//mpu6050_dmp_enable(hi2c);

	remaining_bytes = length;

	while (remaining_bytes) {
		if (remaining_bytes < 256) {
			bytes_to_write = remaining_bytes;
		} else {
			bytes_to_write = 256;
		}
		//write_mem(hi2c, write_ptr, bytes_to_write, bank, 0, false);
		CHECK_AND_HANDLE_ERROR(set_memory_bank(hi2c, bank, false, false));
		CHECK_AND_HANDLE_ERROR(set_memory_start_address(hi2c, 0));

		//uint8_t f_buf[16] = {0x11, 0x22, 0, };
#if 1
		if (mpu6050_write(hi2c, MPU6050_REG_MEM_R_W, write_ptr, bytes_to_write) != MPU6050_OK) {
			return MPU6050_ERR;
		}
#endif

		//CHECK_AND_HANDLE_ERROR(mpu6050_write(hi2c, MPU6050_REG_MEM_R_W, write_ptr, bytes_to_write));

		if (verify) {
			CHECK_AND_HANDLE_ERROR(set_memory_bank(hi2c, bank, false, false));
			CHECK_AND_HANDLE_ERROR(set_memory_start_address(hi2c, 0));

			CHECK_AND_HANDLE_ERROR(mpu6050_read(hi2c, MPU6050_REG_MEM_R_W, (uint8_t*)&verify_buf[0], bytes_to_write));

			if (memcmp(write_ptr, verify_buf, bytes_to_write)) {
				return MPU6050_MEM_WRITE_FAIL;
			}
		}


		write_ptr += bytes_to_write;

		remaining_bytes = remaining_bytes - bytes_to_write;
		bank += 1;
	}

	return MPU6050_OK;

}

mpu6050_status_t is_mpu6050_sleeping(I2C_HandleTypeDef *hi2c, uint8_t *is_sleeping) {
	uint8_t pwr_mgmt_1  = 0;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_PWMGMT_1, &pwr_mgmt_1) != MPU6050_OK) {
		return MPU6050_ERR;
	}
		*is_sleeping = (pwr_mgmt_1 >> 6) & 0x01;

		return MPU6050_OK;

}
mpu6050_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr) {

	mpu6050_i2c_addr = i2c_dev_addr;

	CHECK_AND_HANDLE_ERROR(mpu6050_reset(hi2c));

	HAL_Delay(100);

	uint8_t read_byte  = 0;
	if (mpu6050_read_byte(hi2c, MPU6050_REG_WHOAMI, &read_byte) != MPU6050_OK) {
		return MPU6050_ERR;
	}

#if 0
	if (read_byte == 0x68 || read_byte == 0x98) {
		printf("Valid mpu6050 found at address %X\n", mpu6050_i2c_addr);
	} else {
		printf("Invalid device found at address %X\n", mpu6050_i2c_addr);
		return MPU6050_ERR;
	}
#endif

	if ( !(read_byte == 0x68 || read_byte == 0x98)) {
		return MPU6050_ERR;
	}


	uint8_t data = 0x00;
	if (mpu6050_write_byte(hi2c, MPU6050_REG_PWMGMT_1, data) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	return MPU6050_OK;
}





mpu6050_status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr,\
		                   mpu6050_accel_data_t *accel_data) {

	uint8_t raw_data[6];
	mpu6050_status_t status = mpu6050_read(hi2c, MPU6050_REG_ACCEL_START, raw_data, sizeof(raw_data));
	if (status != MPU6050_OK)
		return status;

	accel_data->x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	accel_data->y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
	accel_data->z = (int16_t)(raw_data[4] << 8 | raw_data[5]);

	return MPU6050_OK;
}



mpu6050_accel_data_t mpu6050_accelerometer_calibration(const mpu6050_accel_data_t *error_offset, \
						mpu6050_accel_data_t *raw_data)
{

	mpu6050_accel_data_t accel_calibrated;
	accel_calibrated.x = raw_data->x - error_offset->x;
	accel_calibrated.y = raw_data->y - error_offset->y;
	accel_calibrated.z = raw_data->z - error_offset->z;

	return accel_calibrated;

}




mpu6050_status_t mpu6050_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf \
)
{

	uint8_t value = 0;

	if (mpu6050_read_byte(hi2c, MPU6050_REG_CONFIG, &value) != MPU6050_OK){
		return MPU6050_ERR;
	}

	value &= ~(0x7);
	value |= (uint8_t)dlpf;
	if (mpu6050_write_byte(hi2c, MPU6050_REG_CONFIG, value) != MPU6050_OK){
		return MPU6050_ERR;
	}

	return MPU6050_OK;
}



static mpu6050_status_t set_memory_bank(I2C_HandleTypeDef *hi2c, uint8_t bank, bool prefetch_enabled, bool user_bank) {
	bank &= 0x1F; //extract the first 5 bits
	if (user_bank) bank |= 0x20;
	if (prefetch_enabled) bank |= 0x40;
	return mpu6050_write_byte(hi2c, MPU6050_REG_BANK_SEL, bank);
}


 mpu6050_status_t mpu6050_set_memory_bank(I2C_HandleTypeDef *hi2c, uint8_t bank, bool prefetch_enabled, bool user_bank) {
	bank &= 0x1F; //extract the first 5 bits
	if (user_bank)
		bank |= 0x20;
	if (prefetch_enabled)
			bank |= 0x40;
	return mpu6050_write_byte(hi2c, MPU6050_REG_BANK_SEL, bank);
}


static mpu6050_status_t set_memory_start_address(I2C_HandleTypeDef *hi2c, uint8_t mem_start_addr) {
	return mpu6050_write_byte(hi2c, MPU6050_REG_MEM_START_ADDR, mem_start_addr);
}

 mpu6050_status_t mpu6050_set_memory_start_address(I2C_HandleTypeDef *hi2c, uint8_t mem_start_addr) {
	return mpu6050_write_byte(hi2c, MPU6050_REG_MEM_START_ADDR, mem_start_addr);
}

 mpu6050_status_t mpu6050_read_memory_byte(I2C_HandleTypeDef *hi2c, uint8_t* data) {
	return mpu6050_read_byte(hi2c, MPU6050_REG_MEM_R_W, data);
}

 mpu6050_status_t mpu6050_read_otp_valid_bit(I2C_HandleTypeDef *hi2c, uint8_t* otp) {

	uint8_t read_byte = 0;
	CHECK_AND_HANDLE_ERROR(mpu6050_read_byte(hi2c, MPU6050_REG_XG_OFFS_TC, &read_byte));
	*otp = read_byte & 0x01;
	return MPU6050_OK;
}


static mpu6050_status_t get_interrupt_status(I2C_HandleTypeDef *hi2c, uint8_t *data) {
	return mpu6050_read_byte(hi2c, MPU6050_REG_INT_STATUS, data);
}

static mpu6050_status_t get_interrupt_setting(I2C_HandleTypeDef *hi2c, uint8_t *data) {
	return mpu6050_read_byte(hi2c, MPU6050_REG_INT_EN, data);
}



void mpu6050_interrupt_handle(I2C_HandleTypeDef *hi2c) {
	uint8_t int_status = 0;
	uint8_t int_settings;
	get_interrupt_status(hi2c, &int_status);
	get_interrupt_setting(hi2c, &int_settings);
	if ( (int_settings & FF_INT) && (int_status & FF_INT) ) {
		mpu6050_free_fall_detection_callback();
	} else if ((int_settings & MOT_INT) && (int_status & MOT_INT)) {
		mpu6050_motion_detection_callback();
	} else if ((int_settings & ZMOT_INT) && (int_status & ZMOT_INT)){
		mpu6050_zero_motion_detection_callback();
	} else if ((int_settings & RAW_RDY_INT) && (int_status & RAW_RDY_INT)) {
		mpu6050_raw_data_ready_callback();
	} else if ((int_settings & DMP_INT) && (int_status & DMP_INT)) {
		mpu6050_dmp_callabck();
	} else {
		//TODO
	}
}


__weak void mpu6050_free_fall_detection_callback(void) {

}


__weak void mpu6050_motion_detection_callback(void) {

}


__weak void mpu6050_zero_motion_detection_callback(void) {

}


__weak void mpu6050_raw_data_ready_callback(void) {

}

__weak void mpu6050_dmp_callabck(void) {

}


void mpu6050_set_slave_address(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t address) {
	if (num > 3) return;
	mpu6050_write_byte(hi2c, MPU6050_RA_I2C_SLV0_ADDR + (num*3), address);
}

mpu6050_status_t mpu6050_i2c_master_mode_disable(I2C_HandleTypeDef *hi2c) {
	uint8_t user_ctrl = 0;
	CHECK_AND_HANDLE_ERROR(mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl));
	user_ctrl &= ~(1 << 5);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);
}

mpu6050_status_t mpu6050_reset_i2c_master(I2C_HandleTypeDef *hi2c) {
	uint8_t user_ctrl = 0;
	CHECK_AND_HANDLE_ERROR(mpu6050_read_byte(hi2c, MPU6050_REG_USER_CTRL, &user_ctrl));
	user_ctrl |= (1 << 1);
	return mpu6050_write_byte(hi2c, MPU6050_REG_USER_CTRL, user_ctrl);
}

mpu6050_status_t mpu6050_set_clock_source(I2C_HandleTypeDef *hi2c, uint8_t source) {
	uint8_t pw_mgmt_1 = 0;
	pw_mgmt_1 |= (source & 0x7);
	return mpu6050_write_byte(hi2c, MPU6050_REG_PWMGMT_1, pw_mgmt_1);
}

mpu6050_status_t mpu6050_set_sample_rate(I2C_HandleTypeDef *hi2c, int8_t rate) {
	return mpu6050_write_byte(hi2c, MPU6050_RA_SMPLRT_DIV, rate);
}

mpu6050_status_t mpu6050_set_external_frame_sync(I2C_HandleTypeDef *hi2c, uint8_t sync) {
	uint8_t config = 0;
	CHECK_AND_HANDLE_ERROR(mpu6050_read_byte(hi2c, MPU6050_RA_CONFIG, &config));
	config |= ((sync & 0x7) << 3);
	return mpu6050_write_byte(hi2c, MPU6050_RA_CONFIG, config);
}

mpu6050_status_t mpu6050_set_full_scale_gyro_range(I2C_HandleTypeDef *hi2c, uint8_t range) {
	uint8_t gyro_config = 0;
	gyro_config |= ((range & 0x3) << 3);
	return mpu6050_write_byte(hi2c, MPU6050_RA_GYRO_CONFIG, gyro_config);
}

mpu6050_status_t mpu6050_get_fifo_count(I2C_HandleTypeDef *hi2c, uint16_t *fifoCount) {
    uint8_t buffer[2];
    CHECK_AND_HANDLE_ERROR(mpu6050_read(hi2c, MPU6050_REG_FIFO_COUNT_H, buffer, 2 ));
    *fifoCount = ((uint16_t)buffer[0] << 8) | buffer[1];
    return MPU6050_OK;
}

mpu6050_status_t MPU6050_read_fifo(I2C_HandleTypeDef *hi2c, uint8_t *data, uint16_t length) {
	CHECK_AND_HANDLE_ERROR(mpu6050_read(hi2c, MPU6050_REG_FIFO_R_W, data, length ));
	return MPU6050_OK;
}

/*
 * DMP Packet Structure

 */
mpu6050_status_t mpu6050_get_dmp_packet(I2C_HandleTypeDef *hi2c, mpu6050_dmp_packet_t *packet) {
	//1. read fifo
	//2. destructure packet

}



