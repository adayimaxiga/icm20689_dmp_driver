/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "icm20689Setup.h"
#include "icm20689Defs.h"
#include "icm20689.h"
#include "icm20689Dmp3Driver.h"
#include "icm20689DataConverter.h"

#include "DataConverter.h"
#include "Message.h"

static int serif_write(struct inv_icm20689 * s, uint16_t reg, uint32_t length, const uint8_t *data)
{
	return inv_icm20689_write_reg(s, (uint8_t)reg, data, length);
}

static int serif_read(struct inv_icm20689 * s, uint16_t reg, uint32_t length, uint8_t *data)
{
	return inv_icm20689_read_reg(s, (uint8_t)reg, data, length);
}

int inv_icm20689_get_whoami(struct inv_icm20689 * s, uint8_t * whoami)
{
	return inv_icm20689_read_reg_one(s, REG_WHO_AM_I, whoami);
}

int inv_icm20689_get_chip_info(struct inv_icm20689 * s, uint8_t chip_info[3])
{
	int result = inv_icm20689_read_reg_one(s, REG_WHO_AM_I, &chip_info[0]);

#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
	result |= inv_icm20689_read_reg_one(s, MPUREG_MANUFACTURER_ID, &chip_info[1]);
	result |= inv_icm20689_read_reg_one(s, MPUREG_CHIP_ID, &chip_info[2]);
#else
	chip_info[1] = 0;
	chip_info[2] = 0;
#endif

	return result;
}

int inv_icm20689_set_dmp_address(struct inv_icm20689 * s)
{
	int result;
	unsigned char dmp_cfg[2] = {0};
	unsigned short config;

	// Write DMP Start address
	inv_icm20689_get_dmp_start_address(s, &config);
	/* setup DMP start address and firmware */
	dmp_cfg[0] = (unsigned char)((config >> 8) & 0xff);
	dmp_cfg[1] = (unsigned char)(config & 0xff);

	result = inv_icm20689_mems_write_reg(s, REG_PRGM_START_ADDRH, 2, dmp_cfg);
	return result;
}

void inv_icm20689_init_matrix(struct inv_icm20689 * s)
{
	// initialize chip to body
	s->s_quat_chip_to_body[0] = (1L<<30);
	s->s_quat_chip_to_body[1] = 0;
	s->s_quat_chip_to_body[2] = 0;
	s->s_quat_chip_to_body[3] = 0;
	//initialize mounting matrix
	memset(s->mounting_matrix, 0, sizeof(s->mounting_matrix));
	s->mounting_matrix[0] = 1;
	s->mounting_matrix[4] = 1;
	s->mounting_matrix[8] = 1;
	//initialize soft iron matrix
	s->soft_iron_matrix[0] = (1L<<30);
	s->soft_iron_matrix[4] = (1L<<30);
	s->soft_iron_matrix[8] = (1L<<30);

	inv_icm20689_set_chip_to_body_axis_quaternion(s, s->mounting_matrix, 0.0);
}

static void DmpDriver_convertion(signed char transformedtochar[9], const int32_t MatrixInQ30[9])
{
	// To convert Q30 to signed char value
	uint8_t iter;
	for (iter = 0; iter < 9; ++iter)
	transformedtochar[iter] = MatrixInQ30[iter] >> 30;
}

int inv_icm20689_set_matrix(struct inv_icm20689 * s, float matrix[9], enum inv_icm20689_sensor sensor)
{
	int32_t mounting_mq30[9];
	int result = 0;
	int i;

	if ((sensor == INV_icm20689_SENSOR_RAW_ACCELEROMETER) ||
		/*(sensor == INV_icm20689_SENSOR_ACCELEROMETER) ||*/
		(sensor == INV_icm20689_SENSOR_RAW_GYROSCOPE) ||
		(sensor == INV_icm20689_SENSOR_GYROSCOPE) ||
		(sensor == INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED)) {
		for(i = 0; i < 9; ++i)
		mounting_mq30[i] = (int32_t)(matrix[i] * (1 << 30));
		// Convert mounting matrix in char
		DmpDriver_convertion(s->mounting_matrix, mounting_mq30);
		// Apply new matrix
		inv_icm20689_set_chip_to_body_axis_quaternion(s, s->mounting_matrix, 0.0);
	}

	return result;
}

int inv_icm20689_initialize(struct inv_icm20689 * s, const unsigned char *dmp_image_sram)
{
	int result = 0;
	uint8_t data = 0;
	unsigned i;

	// Set variables to default values
	memset(&s->base_state, 0, sizeof(s->base_state));

	s->base_state.lp_en_support = 1;
	// Do not allow DMP to execute in accel low power mode
	s->base_state.pwr_mgmt_2 = BIT_DMP_LP_DIS | BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY;
	s->power_state = PowerStateSleepState;
	s->wom_enabled = 0;

	for(i = 0; i < sizeof(s->requested_odr)/sizeof(s->requested_odr[0]); ++i) {
		s->requested_odr[i] = ODR_MIN_DELAY;
	}

	result |= inv_icm20689_soft_reset(s);

	s->base_state.pwr_mgmt_1 = CLK_SEL;
	result |= serif_write(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);

	if(s->serif.is_spi) {
		s->base_state.user_ctrl |= BIT_I2C_IF_DIS;
	}
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &s->base_state.user_ctrl);

	result |= serif_write(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

	result |= inv_icm20689_mems_read_reg(s, MPUREG_WHO_AM_I, 1, &data);

    //Setup DMP.
    result |= inv_icm20689_load_firmware(s, dmp_image_sram);
    if(result)
	    return result;
    else
		s->base_state.firmware_loaded = 1;
	
    result |= inv_icm20689_set_dmp_address(s);
	
    // Enable Interrupts.
    data = BIT_FIFO_OVERFLOW_EN | BIT_DMP_INT_EN;
    result |= inv_icm20689_mems_write_reg(s, MPUREG_INT_ENABLE, 1, &data); // Enable DMP Interrupt

	// Setup MEMs properties.
	s->base_state.accel_averaging = 1; //Change this value if higher sensor sample averaging is required.
	s->base_state.gyro_averaging = 1;  //Change this value if higher sensor sample averaging is required.

	// Initial sampling rate BASE_SAMPLE_RATE/(SAMPLE_RATE_DIVIDER+1) = 1000/(4+1) = 200Hz
	// DMP always runs at 200Hz so keep this divider unchanged
	inv_icm20689_set_divider(s, SAMPLE_RATE_DIVIDER);       

	result |= inv_icm20689_set_gyro_fullscale(s, MPU_FS_2000dps);
	result |= inv_icm20689_set_accel_fullscale(s, MPU_FS_4G);

	// set acc bw at 420 to be inline with A_DPLPF_CFG startup value
	result |= inv_icm20689_set_accel_bandwidth(s, MPU_ABW_420);
	// set gyr bw at 176 to be inline with first config
	result |= inv_icm20689_set_gyro_bandwidth(s, MPU_GBW_176);

	data = BITS_FIFO_SIZE; // 512B for 20789 using DMP
	result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &data);
	
	return result;
}

int inv_icm20689_soft_reset(struct inv_icm20689 * s)
{
	int result = 0;
	int timeout = 1000; /* 1s */

	// Reset the internal registers and restores the default settings.
	// The bit automatically clears to 0 once the reset is done.
	s->base_state.pwr_mgmt_1 = BIT_DEVICE_RESET;
	result |= serif_write(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
	if(result)
		return result;

	inv_icm20689_sleep(50); // wait for 50ms after soft reset

	do {
		result = serif_read(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
		if(result)
			return result;

		inv_icm20689_sleep(1);
		timeout -= 1;

		if(timeout < 0)
			return INV_ERROR_TIMEOUT;
	} while (s->base_state.pwr_mgmt_1 != RST_VAL_PWR_MGMT_1); // this is the default expected value

	return 0;
}

int inv_icm20689_set_divider(struct inv_icm20689 * s, uint8_t idiv)
{
	int result = 0;

	s->base_state.gyro_div = idiv;
	s->base_state.accel_div = idiv;
	s->base_state.compass_div = idiv;

	result |= inv_icm20689_mems_write_reg(s, MPUREG_SMPLRT_DIV, 1, &idiv);
	return result;
}

/** @brief     This function sets the power state of the Scorpion+ chip loop
 *  @param[in] func   CHIP_AWAKE, CHIP_LP_ENABLE
 *  @param[in] on_off The functions are enabled if previously disabled and
 *                    disabled if previously enabled based on the value of On/Off.
 */
int inv_icm20689_set_chip_power_state(struct inv_icm20689 * s, uint8_t func, uint8_t on_off)
{
	int status = 0;

	switch(func) {
	case CHIP_AWAKE:
	{
		if(on_off) {
			if((s->base_state.wake_state & CHIP_AWAKE) == 0) {
				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP; // clear sleep bit
				status = serif_write(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				inv_icm20689_sleep_us(100); // after writing the bit wait 100 Micro Seconds
			}
		}
		else {
			if(s->base_state.wake_state & CHIP_AWAKE) {
				s->base_state.pwr_mgmt_1 |= BIT_SLEEP; // set sleep bit
				status = serif_write(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_AWAKE;
				inv_icm20689_sleep_us(100); // after writing the bit wait 100 Micro Seconds
			}
		}
	}
	break;

	case CHIP_LP_ENABLE:
	{
		if(s->base_state.lp_en_support == 1) {
			if(on_off) {
				if((s->base_state.wake_state & CHIP_LP_ENABLE) == 0) {
					s->base_state.pwr_mgmt_2 &= ~BIT_LP_DIS; // enable lp
					//s->base_state.pwr_mgmt_2 &= ~BIT_DMP_LP_DIS; // lp_en ON
					status = serif_write(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
					s->base_state.wake_state |= CHIP_LP_ENABLE;
					inv_icm20689_sleep_us(100); // after writing the bit wait 100 Micro Seconds
				}
			}
			else {
				if((s->base_state.wake_state & CHIP_LP_ENABLE)){
					s->base_state.pwr_mgmt_2 |= BIT_LP_DIS; // disable lp
					//s->base_state.pwr_mgmt_2 |= BIT_DMP_LP_DIS; // lp_en off
					status = serif_write(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
					s->base_state.wake_state &= ~CHIP_LP_ENABLE;
					inv_icm20689_sleep_us(100); // after writing the bit wait 100 Micro Seconds
				}
			}
		}
	}
	break;

	default:
		break;
	} // end switch

	return status;
}

uint8_t inv_icm20689_get_chip_power_state(struct inv_icm20689 * s)
{
	return s->base_state.wake_state;
}

uint16_t inv_icm20689_get_chip_base_sample_rate(struct inv_icm20689 * s)
{
	(void)s;

	return BASE_SAMPLE_RATE; // base_state.sample_rate;
}

int inv_icm20689_accel_fsr_2_reg(int32_t fsr)
{
	switch(fsr) {
	case 2000:  return MPU_FS_2G;
	case 4000:  return MPU_FS_4G;
	case 8000:  return MPU_FS_8G;
	case 16000: return MPU_FS_16G;
	default:    return NUM_MPU_AFS;
	}
}

int inv_icm20689_reg_2_accel_fsr(uint8_t reg)
{
	switch(reg) {
	case MPU_FS_2G:   return 2000;
	case MPU_FS_4G:   return 4000;
	case MPU_FS_8G:   return 8000;
	case MPU_FS_16G:  return 16000;
	default:          return -1;
	}
}

int inv_icm20689_set_accel_fullscale(struct inv_icm20689 * s, int level)
{
	if(level >= NUM_MPU_AFS)
		return INV_ERROR_BAD_ARG;

	s->base_state.accel_fullscale = level;
	return 0;
}

int inv_icm20689_set_accel_bandwidth(struct inv_icm20689 * s, int level)
{
	if(level >= NUM_MPU_ABW)
		return INV_ERROR_BAD_ARG;

	s->base_state.accel_bw = level;
	return 0;
}

uint8_t inv_icm20689_get_accel_fullscale(struct inv_icm20689 * s)
{
	return s->base_state.accel_fullscale;
}

uint16_t inv_icm20689_get_accel_bandwidth(struct inv_icm20689 * s)
{
	return s->base_state.accel_bw;
}

int inv_icm20689_gyro_fsr_2_reg(int32_t fsr)
{
	switch(fsr) {
	case 250:  return MPU_FS_250dps;
	case 500:  return MPU_FS_500dps;
	case 1000: return MPU_FS_1000dps;
	case 2000: return MPU_FS_2000dps;
#if (MEMS_CHIP == HW_ICM20690)
	case 31: 
	case 32:   return MPU_FS_31dps;
	case 62:   
	case 63:   return MPU_FS_62dps;
	case 125:  return MPU_FS_125dps;
#endif
	default:   return NUM_MPU_GFS;
	}
}

int inv_icm20689_reg_2_gyro_fsr(uint8_t reg)
{
	switch(reg) {
	case MPU_FS_250dps:  return 250;
	case MPU_FS_500dps:  return 500;
	case MPU_FS_1000dps: return 1000;
	case MPU_FS_2000dps: return 2000;
#if (MEMS_CHIP == HW_ICM20690)
	case MPU_FS_31dps:   return 31;
	case MPU_FS_62dps:   return 62;
	case MPU_FS_125dps:  return 125;
#endif
	default:             return -1;
	}
}

int inv_icm20689_set_gyro_fullscale(struct inv_icm20689 * s, int level)
{
	if(level >= NUM_MPU_GFS)
		return INV_ERROR_BAD_ARG;

	s->base_state.gyro_fullscale = level;
	return 0;
}

int inv_icm20689_set_gyro_bandwidth(struct inv_icm20689 * s, int level)
{
	if(level >= NUM_MPU_GBW)
		return INV_ERROR_BAD_ARG;

	s->base_state.gyro_bw = level;

	return 0;
}

uint8_t inv_icm20689_get_gyro_fullscale(struct inv_icm20689 * s)
{
	return s->base_state.gyro_fullscale;
}

uint16_t inv_icm20689_get_gyro_bandwidth(struct inv_icm20689 * s)
{
	return s->base_state.gyro_bw;
}

int inv_icm20689_is_advanced_features_supported(void)
{
#if (MEMS_CHIP == HW_ICM20690)
	// only the icm20690 supports OIS sensor and advanced FSYNC feature
	return 1;
#else
	return 0;
#endif
}

int inv_icm20689_set_fsync_bit_location(struct inv_icm20689 * s, int bit_location)
{
	int result = 0;
#if (MEMS_CHIP == HW_ICM20690)
	uint8_t bank_sel, config_reg;

	// set as CFG bank
	bank_sel = BIT_CONFIG_BANK;
	result |= inv_icm20689_mems_write_reg(s, REG_CFG_USER_BANK_SEL, 1, &bank_sel);

	// enable the FSYNC ODR delay counter read by external interface - needed as this bit is set to 1 by default (HW related issue)
	result |= inv_icm20689_mems_read_reg(s, REG_INTOSC_CTRL_NEW, 1, &config_reg);
	config_reg &= ~BIT_DELAY_TIME_RD_DISABLE;
	result |= inv_icm20689_mems_write_reg(s, REG_INTOSC_CTRL_NEW, 1, &config_reg);

	// enable the FSYNC ODR delay counter configuration 
	result |= inv_icm20689_mems_read_reg(s, MPUREG_ACC_CTRL_NEW0, 1, &config_reg);
	config_reg |= BIT_ODR_DLY_TIME_EN_CFG;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_ACC_CTRL_NEW0, 1, &config_reg);

	// set USR bank
	bank_sel = BIT_USER_BANK;
	result |= inv_icm20689_mems_write_reg(s, REG_CFG_USER_BANK_SEL, 1, &bank_sel);

	// enable the FSYNC ODR delay counter
	result |= inv_icm20689_mems_read_reg(s, MPUREG_ANA_CTRL_NEW_1, 1, &config_reg);
	config_reg |= BIT_ODR_DELAY_TIME_EN;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_ANA_CTRL_NEW_1, 1, &config_reg);

	// clear ext_sync_set field bits[5:3]
	result |= inv_icm20689_mems_read_reg(s, MPUREG_CONFIG, 1, &config_reg);
	config_reg &= ~BITS_EXT_SYNC_SET;

	// enable fsync pin data to be toggled
	// bits[5:3] = 0: Disable FSYNC pin data to be sampled
	//             1: Enables the FSYNC pin data to be sampled at TEMP_OUT_L[0]
	config_reg |= bit_location << BIT_POS_EXT_SYNC_SET;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_CONFIG, 1, &config_reg);
#endif
	(void)s, (void)bit_location;

	return result;
}
