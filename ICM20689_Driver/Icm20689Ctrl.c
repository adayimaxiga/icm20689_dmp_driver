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

#include "icm20689Ctrl.h"
#include "icm20689Defs.h"
#include "icm20689.h"
#include "icm20689MPUFifoControl.h"
#include "icm20689Dmp3Driver.h"
#include "icm20689DataConverter.h"

// Determines which base sensor needs to be on based upon s->enabled_sensor_mask
#define INV_NEEDS_ACCEL_MASK    (/*1L<<INV_icm20689_SENSOR_ACCELEROMETER | */\
                                 1L<<INV_icm20689_SENSOR_RAW_ACCELEROMETER | \
								 1L<<INV_icm20689_SENSOR_GAME_ROTATION_VECTOR)
								  
#define INV_NEEDS_GYRO_MASK     (1L<<INV_icm20689_SENSOR_GYROSCOPE | \
                                 1L<<INV_icm20689_SENSOR_RAW_GYROSCOPE | \
								 1L<<INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED | \
								 1L<<INV_icm20689_SENSOR_GAME_ROTATION_VECTOR)
								 
#define INV_NEEDS_TEMP_MASK     (1L<<ANDROID_SENSOR_TEMPERATURE)

#define GYRO_AVAILABLE      0x01
#define ACCEL_AVAILABLE     0x02
#define HIGH_ODR_REQUESTED  0x10
#define TEMP_AVAILABLE      0x20


/* Convert public sensor type for icm20689 to internal sensor id (Android)
 */

static uint32_t is_icm20689_sensor_enabled(struct inv_icm20689 * s, uint8_t icm20689_sensor)
{
	return (s->enabled_sensor_mask & (1L << icm20689_sensor));
}

static int apply_accel_fsr(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t accel_config_1_reg;

	// Set FSR
	result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_CONFIG, 1, &accel_config_1_reg);
	accel_config_1_reg &= ~BITS_ACCEL_FS_SEL;//0xE7;
	accel_config_1_reg |= (s->base_state.accel_fullscale << BIT_POS_ACCEL_FS_SEL);
	result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_CONFIG, 1, &accel_config_1_reg);

	return result;
}

static int apply_accel_bw(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t accel_config_2_reg;
	uint8_t dec2_cfg;

	// Set averaging
	switch(s->base_state.accel_averaging) {
	case 1:  dec2_cfg = 0; break;
	case 4:  dec2_cfg = 0; break;
	case 8:  dec2_cfg = 1; break;
	case 16: dec2_cfg = 2; break;
	case 32: dec2_cfg = 3; break;
	default: dec2_cfg = 0; break;
	}

	result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &accel_config_2_reg);
	accel_config_2_reg &= ~BITS_DEC2_CFG;
	accel_config_2_reg &= ~BITS_A_DLPF_CFG;

	// Set DEC2 and A_DLPF_CFG
	if(inv_icm20689_get_chip_power_state(s) & CHIP_LP_ENABLE) // accel LP mode
	{
		accel_config_2_reg |= 7; // A_DLPF_CFG = 7 
		accel_config_2_reg |= (dec2_cfg << BIT_POS_DEC2_CFG);
	}
	else // accel LN mode
	{
		if(s->base_state.accel_bw > 6)
			accel_config_2_reg |= MPU_ABW_218; // 0 < A_DLPF_CFG < 7
		else
			accel_config_2_reg |= s->base_state.accel_bw;
		dec2_cfg = 0;		
		accel_config_2_reg |= (dec2_cfg << BIT_POS_DEC2_CFG);
	}
	result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &accel_config_2_reg);

	return result;
}

static int apply_accel_bias(struct inv_icm20689 * s)
{
	int i, axis_sign;
	int rc = -1;
	uint16_t accel_reg_otp[3], sensor_bias[3];
	uint8_t axis, d[2];

	// test if self-test bias has been computed
	if (!s->selftest_done)
		return 0;

	if(!s->accel_reg_bias_computed) {
		// read accelerometer bias registers
		rc = inv_icm20689_mems_read_reg(s, MPUREG_XA_OFFS_H, 2, d);
		accel_reg_otp[0] = ((uint16_t)d[0] << 8) | d[1];
		rc = inv_icm20689_mems_read_reg(s, MPUREG_YA_OFFS_H, 2, d);
		accel_reg_otp[1] = ((uint16_t)d[0] << 8) | d[1];
		rc = inv_icm20689_mems_read_reg(s, MPUREG_ZA_OFFS_H, 2, d);
		accel_reg_otp[2] = ((uint16_t)d[0] << 8) | d[1];

		// determine gravity axis (by default on LN values)
		axis = 0;
		axis_sign = 1;
		if (INV_ABS(s->accel_ois_st_bias[1]) > INV_ABS(s->accel_ois_st_bias[0]))
			axis = 1;
		if (INV_ABS(s->accel_ois_st_bias[2]) > INV_ABS(s->accel_ois_st_bias[axis]))
			axis = 2;
		if(s->accel_ois_st_bias[axis] < 0)
			axis_sign = -1;

		// compute register bias value according to power mode (LP/LN)
		for(i = 0; i < 3; i++) {
			// convert from 2g to 16g (1LSB = 0.49mg) on 15bits register and mask bit0
			// note : there is no need to shift the register value as the register is 0.98mg steps
			s->accel_reg_bias[i] = ((uint16_t)(accel_reg_otp[i] - (s->accel_st_bias[i] / 8)) & ~1);
			s->accel_ois_reg_bias[i] = ((uint16_t)(accel_reg_otp[i] - (s->accel_ois_st_bias[i] / 8)) & ~1);
			// remove gravity for axis
			if(i == axis) {
				s->accel_reg_bias[i] += ((DEF_ST_SCALE / DEF_ST_ACCEL_FS) / 8) * axis_sign; 
				s->accel_ois_reg_bias[i] += ((DEF_ST_SCALE / DEF_ST_ACCEL_FS) / 8) * axis_sign; 
			}
		}
		s->accel_reg_bias_computed = 1;
	}

	//get bias value according to power mode
	if((s->power_state == PowerStateAccLPState) || (s->power_state == PowerState6AxisLPState)) {
		for(i = 0; i < 3; i++)
			sensor_bias[i] = s->accel_reg_bias[i];
	} else if((s->power_state == PowerStateAccLNState) || (s->power_state == PowerState6AxisLNState)) {
		for(i = 0; i < 3; i++)
			sensor_bias[i] = s->accel_ois_reg_bias[i];
	} else
		return rc;

	d[0] = (sensor_bias[0] >> 8) & 0xff;
	d[1] = (sensor_bias[0] & 0xff);
	rc = inv_icm20689_mems_write_reg(s, MPUREG_XA_OFFS_H, 2, d);

	d[0] = (sensor_bias[1] >> 8) & 0xff;
	d[1] = sensor_bias[1] & 0xff;
	rc |= inv_icm20689_mems_write_reg(s, MPUREG_YA_OFFS_H, 2, d);

	d[0] = (sensor_bias[2] >> 8) & 0xff;
	d[1] = sensor_bias[2] & 0xff;
	rc |= inv_icm20689_mems_write_reg(s, MPUREG_ZA_OFFS_H, 2, d);

	return rc;
}

static int apply_gyr_fsr_bw(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t data;
	uint8_t gyro_config_1_reg;
	uint8_t g_avgcfg;

	// Set FSR
	result |= inv_icm20689_mems_read_reg(s, MPUREG_GYRO_CONFIG, 1, &gyro_config_1_reg);
	gyro_config_1_reg &= ~BITS_GYRO_FS_SEL;
	gyro_config_1_reg |= (inv_icm20689_get_gyro_fullscale(s)<< BIT_POS_GYRO_FS_SEL);
	result |= inv_icm20689_mems_write_reg(s, MPUREG_GYRO_CONFIG, 1, &gyro_config_1_reg);

	// Set averaging
	switch(s->base_state.gyro_averaging) {
	case 1:   g_avgcfg = 0; break;
	case 2:   g_avgcfg = 1; break;
	case 4:   g_avgcfg = 2; break;
	case 8:   g_avgcfg = 3; break;
	case 16:  g_avgcfg = 4; break;
	case 32:  g_avgcfg = 5; break;
	case 64:  g_avgcfg = 6; break;
	case 128: g_avgcfg = 7; break;
	default:  g_avgcfg = 0; break;
	}

	result |= inv_icm20689_mems_read_reg(s, MPUREG_LP_CONFIG, 1, &data);
	data &= ~BITS_G_AVGCFG;
	data |= (g_avgcfg<<BIT_POS_G_AVGCFG);
	result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &data);

	// Set DLPF_CFG
	result |= inv_icm20689_mems_read_reg(s, MPUREG_CONFIG, 1, &data);
	data &= ~BITS_DLPF_CFG; // Clear bits[2:0]
	data |= inv_icm20689_get_gyro_bandwidth(s);
	result |= inv_icm20689_mems_write_reg(s, MPUREG_CONFIG, 1, &data);

	return result;
}

static int apply_gyr_bias(struct inv_icm20689 * s)
{
	int i;
	int rc = -1;
	int sensor_bias[3];
	uint8_t data[2];

	// test if self-test bias has been computed
	if (!s->selftest_done)
			return 0;

	for(i = 0; i < 3; i++) {

		// get bias value according to power mode
		if((s->power_state == PowerStateGyrLPState) || (s->power_state == PowerState6AxisLPState))
			sensor_bias[i] = s->gyro_st_bias[i];
		else if((s->power_state == PowerStateGyrLNState) || (s->power_state == PowerState6AxisLNState))
			sensor_bias[i] = s->gyro_ois_st_bias[i];
		else
			return rc;

		// convert bias value from 250dps to 1000dps
		sensor_bias[i] = - sensor_bias[i] / 4;
	}

	data[0] = (sensor_bias[0] >> 8) & 0xff;
	data[1] = sensor_bias[0] & 0xff;
	rc = inv_icm20689_mems_write_reg(s, MPUREG_XG_OFFS_USRH, 2, data);

	data[0] = (sensor_bias[1] >> 8) & 0xff;
	data[1] = sensor_bias[1] & 0xff;
	rc |= inv_icm20689_mems_write_reg(s, MPUREG_YG_OFFS_USRH, 2, data);

	data[0] = (sensor_bias[2] >> 8) & 0xff;
	data[1] = sensor_bias[2] & 0xff;
	rc |= inv_icm20689_mems_write_reg(s, MPUREG_ZG_OFFS_USRH, 2, data);

	return rc;
}

int inv_icm20689_check_wom_status(struct inv_icm20689 * s, uint8_t int_status)
{
	int wom_data = 0;

	(void)s;

	if(int_status & BIT_WOM_X_INT)
		wom_data |= 0x1;
	if(int_status & BIT_WOM_Y_INT)
		wom_data |= 0x2;
	if(int_status & BIT_WOM_Z_INT)
		wom_data |= 0x4;

	return wom_data;
}

int inv_icm20689_enable_mems(struct inv_icm20689 * s, int bit_mask, uint16_t smplrt_divider)
{
	int result = 0;

#if (MEMS_CHIP == HW_icm20689)
	if(smplrt_divider == 1)
		bit_mask |= HIGH_ODR_REQUESTED;
#else
	// gyro ODR range in low power mode is 3.91Hz ~ 333.33Hz for 20602 and 20609
	if((smplrt_divider == 1) || ((smplrt_divider == 2) && (bit_mask & GYRO_AVAILABLE)))
		bit_mask |= HIGH_ODR_REQUESTED;
#endif

	switch(bit_mask & (ACCEL_AVAILABLE | GYRO_AVAILABLE | HIGH_ODR_REQUESTED)) {
	case 0 :
		// Procedure 0
		s->base_state.pwr_mgmt_1 |= BIT_SLEEP;
		result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);

		s->base_state.pwr_mgmt_2 |= BIT_PWR_ALL_OFF;
		result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
		
		s->base_state.wake_state &= ~CHIP_AWAKE;
		s->power_state = PowerStateSleepState;
		break;

	case ACCEL_AVAILABLE: // to accel Low Power
		// no need to go above 500Hz data rate so reduce power consumption
		// and put accel in Low Power mode in this case
		switch(s->power_state) 
		{
			case PowerStateSleepState:
				// Procedure 1
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				// change ACCEL_FS_SEL
				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);
				result |= apply_accel_fsr(s);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603. Accel must be in LN mode for 20789 using DMP
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accel in LP
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif

				result |= apply_accel_bw(s);
				inv_icm20689_sleep(20); // accel startup time
				break;
			case PowerStateAccLNState:
				// Procedure 2
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif
				result |= apply_accel_bw(s);
				break;
			case PowerStateGyrLPState:
			case PowerStateGyrLNState:
				//Procedure 4/5
				// change ACCEL_FS_SEL
				result |= apply_accel_fsr(s);

#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif

#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20689_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				result |= apply_accel_bw(s);
				break;
			case PowerState6AxisLPState:
			case PowerState6AxisLNState:
				// Procedure 6/7
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif

#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20689_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				result |= apply_accel_bw(s);
				break;
			default:
				break;
		}

#if ((MEMS_CHIP == HW_ICM20603) || (MEMS_CHIP == HW_icm20689))
		s->power_state = PowerStateAccLNState;
#else
		s->power_state = PowerStateAccLPState;
#endif
		result |= apply_accel_bias(s);
		break;

	case ACCEL_AVAILABLE | HIGH_ODR_REQUESTED: // to accel accel Low Noise
		// accel needs 1kHz data rate since divider is 1 so need to have sample rate of 1kHz
		// which is only possible when accel Low Noise mode is enabled
		switch(s->power_state) 
		{
			case PowerStateSleepState:
				// Procedure 8
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);
				// change ACCEL_FS_SEL
				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);
				inv_icm20689_sleep(20); // accel startup time
				break;
			case PowerStateAccLPState:
				// Procedure 9
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);
				result |= apply_accel_bw(s);
				break;
			case PowerStateGyrLNState:
			case PowerStateGyrLPState:
				// Procedure 11 & 12
				result |= apply_accel_bw(s);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20689_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				break;
			case PowerState6AxisLPState:
			case PowerState6AxisLNState:
				// Procedure 13
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20689_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				break;
			default:
				break;
		}

		s->power_state = PowerStateAccLNState;
		result |= apply_accel_bias(s);
		break;

	case GYRO_AVAILABLE: // to gyro Low Power
		// Turn on gyro only
		switch(s->power_state) 
		{
			case PowerStateSleepState:
				//Procedure 18
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // GYRO_STANDBY = 0 & CYCLE = 0 & SLEEP = 0
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);
				result |= apply_gyr_fsr_bw(s);
				inv_icm20689_sleep(50); // gyro startup time
				break;
			case PowerStateAccLPState:
				//Procedure 19
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);
				result |= apply_gyr_fsr_bw(s);
				break;
			case PowerStateAccLNState:
				//Procedure 20
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			case PowerStateGyrLNState:
				//Procedure 22

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			case PowerState6AxisLPState:
				//Procedure 23
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			case PowerState6AxisLNState:
				//Procedure 24

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			default:
				break;
		}

#if (MEMS_CHIP == HW_ICM20603)
		s->power_state = PowerStateGyrLNState;
#else
		s->power_state = PowerStateGyrLPState;
#endif
		result |= apply_gyr_bias(s);
		break;

	case GYRO_AVAILABLE | HIGH_ODR_REQUESTED: // to Gyro Low Noise

		switch(s->power_state)
		{
			case PowerStateSleepState:
				//Procedure 25
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);

				result |= apply_gyr_fsr_bw(s);
				inv_icm20689_sleep(50); // gyro startup time
				break;
			case PowerStateAccLPState:
				//Procedure 26
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20689_sleep(2);
				result |= apply_gyr_fsr_bw(s);
				break;
			case PowerStateAccLNState:
				//Procedure 27
				result |= apply_gyr_fsr_bw(s);
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			case PowerStateGyrLPState:
				//Procedure 29

				// Gyro full scale should be reconfigured because OIS enabled for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			case PowerState6AxisLPState:
				//Procedure 30

				// Gyro full scale should be reconfigured because OIS enabled for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			case PowerState6AxisLNState:
				//Procedure 31

				// No need to reconfigure Gyro full scale because OIS was already enabled

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			default:
				break;
		}

		s->power_state = PowerStateGyrLNState;
		result |= apply_gyr_bias(s);
		break;

	case ACCEL_AVAILABLE | GYRO_AVAILABLE: // to 6axis Low Power
		// Turn on gyro and accel so accel in low noise mode

		switch(s->power_state)
		{
			case PowerStateSleepState:
				//Procedure 32
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;

				inv_icm20689_sleep(2);

				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);
				result |= apply_gyr_fsr_bw(s);
				inv_icm20689_sleep(50); // gyro startup time
			break;
			case PowerStateAccLPState:
				//Procedure 33
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ALL_OFF; // Zero means all sensors are on
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // gyro on and accelero also, accelero in low-noise
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;

				inv_icm20689_sleep(2);

				result |= apply_gyr_fsr_bw(s);
			break;
			case PowerStateAccLNState:
				//Procedure 34
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
			break;
			case PowerStateGyrLPState:
				//Procedure 36
				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
			break;
			case PowerStateGyrLNState:
				//Procedure 37
				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
			case PowerState6AxisLNState:
				//Procedure 38

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
			default :
			break;
		}

#if (MEMS_CHIP == HW_ICM20603)
		s->power_state = PowerState6AxisLNState;
#else
		s->power_state = PowerState6AxisLPState;
#endif
		result |= apply_accel_bias(s);
		result |= apply_gyr_bias(s);
		break;

	case ACCEL_AVAILABLE | GYRO_AVAILABLE | HIGH_ODR_REQUESTED: // to 6axis Low Noise

		switch(s->power_state) {
		case PowerStateSleepState:
			//Procedure 39
			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

			s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
			s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
			s->base_state.wake_state |= CHIP_AWAKE;
			s->base_state.wake_state &= ~CHIP_LP_ENABLE;
			inv_icm20689_sleep(2);

			apply_accel_fsr(s);
			apply_accel_bw(s);
			apply_gyr_fsr_bw(s);
			inv_icm20689_sleep(50); // gyro startup time
			break;
		case PowerStateAccLPState:
			//Procedure 40
			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

			s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
			s->base_state.wake_state &= ~CHIP_LP_ENABLE;
			inv_icm20689_sleep(2);

			result |= apply_gyr_fsr_bw(s);
			break;
		case PowerStateAccLNState:
			//Procedure 41
			result |= apply_gyr_fsr_bw(s);

			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
			break;
		case PowerStateGyrLPState:
			//Procedure 43
			result |= apply_accel_fsr(s);
			result |= apply_accel_bw(s);

			// Gyro full scale should be reconfigured because OIS enabled for 1st time
			result |= apply_gyr_fsr_bw(s);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
		case PowerStateGyrLNState:
			//Procedure 44
			result |= apply_accel_fsr(s);
			result |= apply_accel_bw(s);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

			// No need to reconfigure Gyro full scale because OIS was already enabled
			break;
		case PowerState6AxisLPState:
			//Procedure 45

			// Gyro full scale should be reconfigured because OIS enabled for 1st time
			result |= apply_gyr_fsr_bw(s);

			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20689_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
		default:
			break;
		}

		s->power_state = PowerState6AxisLNState;
		result |= apply_accel_bias(s);
		result |= apply_gyr_bias(s);
		break;

	case HIGH_ODR_REQUESTED: // no sensor ON
	default:
		break;
	}

	/* workaround to avoid impact on gyro offsets when enable/disable temp (#14339) */
	s->base_state.pwr_mgmt_1 &= ~BIT_TEMP_DISABLE; // Turn ON temp 
	result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);

	return result;
}

/******************/
typedef	struct {
	enum ANDROID_SENSORS     AndroidSensor;
	enum inv_icm20689_sensor InvSensor;
} MinDelayGenElementT;

/******************/

static const MinDelayGenElementT MinDelayGenList[]	= {
	/*{ ANDROID_SENSOR_ACCELEROMETER, INV_icm20689_SENSOR_ACCELEROMETER, },*/
	{ ANDROID_SENSOR_GYROSCOPE, INV_icm20689_SENSOR_GYROSCOPE, },
	{ ANDROID_SENSOR_RAW_ACCELEROMETER, INV_icm20689_SENSOR_RAW_ACCELEROMETER, },
	{ ANDROID_SENSOR_RAW_GYROSCOPE, INV_icm20689_SENSOR_RAW_GYROSCOPE, },
	{ ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED, INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED, },
	{ ANDROID_SENSOR_GAME_ROTATION_VECTOR, INV_icm20689_SENSOR_GAME_ROTATION_VECTOR, }
};

typedef	unsigned	short	MinDelayT;
typedef	unsigned	short	u16;
typedef	unsigned	long	u32;

/******************/
#define	MinDelayGen(s, list)	MinDelayGenActual((s), list, sizeof(list) / sizeof (MinDelayGenElementT))

static MinDelayT MinDelayGenActual(struct inv_icm20689 * s, const MinDelayGenElementT * Element, u32 ElementQuan)
{
	MinDelayT MinDelay = (MinDelayT)-1;

	while(ElementQuan--) {
		if(is_icm20689_sensor_enabled(s, Element->InvSensor)) {
			MinDelayT OdrDelay = s->requested_odr[Element->InvSensor];
			if(MinDelay > OdrDelay) {
				MinDelay = OdrDelay;
			}
		}
		Element++;
	} //	end while elements to process

	return	MinDelay;
}

/******************/
static int set_hw_smplrt_dmp_odrs(struct inv_icm20689 * s, uint16_t * resulting_divider)
{
	int result = 0;
	uint8_t data = 0;
	uint16_t hw_smplrt_divider = 0;

	// Set UI ODR (always <=1kHz)
	// Set accel_fchoice_b to 0
	if(s->enabled_sensor_mask & INV_NEEDS_ACCEL_MASK)
	{
		result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &data);
		data &= ~BITS_ACCEL_FCHOICE_B; // Clear ACCEL_FCHOICE_B
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &data);
	}

	// Set fchoice_b to 0, dlpf_cfg to 1
	if(s->enabled_sensor_mask & INV_NEEDS_GYRO_MASK)
	{
		result |= inv_icm20689_mems_read_reg(s, MPUREG_GYRO_CONFIG, 1, &data);
		data &= ~BITS_FCHOICE_B; // Clear FCHOICE_B
		result |= inv_icm20689_mems_write_reg(s, MPUREG_GYRO_CONFIG, 1, &data);

		result |= inv_icm20689_mems_read_reg(s, MPUREG_CONFIG, 1, &data);
		data &= ~0x7; // Clear bits[2:0]
		data |= 0x1;
		result |= inv_icm20689_mems_write_reg(s, MPUREG_CONFIG, 1, &data);
	}

	// Set sample rate divider
	if((s->enabled_sensor_mask & INV_NEEDS_ACCEL_MASK) ||
	   (s->enabled_sensor_mask & INV_NEEDS_GYRO_MASK) )
	{
		uint16_t minDly;
		// get min delays of all enabled sensors for each sensor engine group

		minDly = MinDelayGen(s, MinDelayGenList);

		// set odrs for each enabled sensors
		if (minDly != 0xFFFF)
		{
			hw_smplrt_divider = SAMPLE_RATE_DIVIDER + 1;
			// Set DMP divider. DMP works when base engine rate = 200Hz so the hw sample rate divider is always 4
			dmp_icm20689_set_sensor_rate(s, INV_SENSOR_ACCEL, 200/(1000/minDly)-1);
			dmp_icm20689_set_sensor_rate(s, INV_SENSOR_GYRO, 200/(1000/minDly)-1);
			dmp_icm20689_set_sensor_rate(s, INV_SENSOR_SIXQ, 200/(1000/minDly)-1);
		}
	}

	*resulting_divider = hw_smplrt_divider;
	return result;
}

int inv_icm20689_set_sensor_period(struct inv_icm20689 * s, enum inv_icm20689_sensor sensor, uint16_t delayInMs)
{
	int result = 0;
	uint16_t resulting_divider = 0;
	uint16_t data_rdy_status = 0;

//	if(delayInMs < ODR_MIN_DELAY) delayInMs = ODR_MIN_DELAY;
//	if(delayInMs > ODR_MAX_DELAY) delayInMs = ODR_MAX_DELAY;

	s->requested_odr[sensor] = delayInMs;

	result |= inv_icm20689_disable_dmp(s);
	result |= inv_icm20689_disable_fifo(s);
	result |= inv_icm20689_reset_fifo(s);

	result |= set_hw_smplrt_dmp_odrs(s, &resulting_divider);

	if (s->enabled_sensor_mask & INV_NEEDS_GYRO_MASK)
		data_rdy_status |= GYRO_AVAILABLE;
	if (s->enabled_sensor_mask & INV_NEEDS_ACCEL_MASK)
		data_rdy_status |= ACCEL_AVAILABLE;
		
	// we reconfigured ODR, so there is a chance that we change from LP to/from LN noise, just check it
	result |= inv_icm20689_enable_mems(s, (int)data_rdy_status, resulting_divider);

	// need to reset and restart FIFO with new configurations
	result |= inv_icm20689_enable_fifo(s);
	result |= inv_icm20689_enable_dmp(s);

	return result;
}

int inv_icm20689_is_sensor_enabled(struct inv_icm20689 * s, enum inv_icm20689_sensor sensor)
{
	if(sensor < INV_icm20689_SENSOR_MAX) {
		return ( ((1<<sensor) & s->enabled_sensor_mask) != 0);
	}

	return 0;
}

int inv_icm20689_enable_sensor(struct inv_icm20689 * s, enum inv_icm20689_sensor sensor, uint8_t enable)
{
	int result = 0;
	uint16_t data_rdy_status = 0;
	uint16_t resulting_divider = 0;
	int data_output_control1_mask = 0;
	int motion_event_control_mask = 0;

	result |= inv_icm20689_disable_dmp(s);
	result |= inv_icm20689_disable_fifo(s);
	result |= inv_icm20689_reset_fifo(s);

	// Update enabled_sensor_mask
	if (enable) {
		s->enabled_sensor_mask |= 1L << sensor; // Set bit
	} else {
		s->enabled_sensor_mask &= ~(1L << sensor); // Clear bit
	}

	// enable DMP features
	if( (s->enabled_sensor_mask & (1<<INV_icm20689_SENSOR_RAW_ACCELEROMETER)) /*|| 
	    (s->enabled_sensor_mask & (1<<INV_icm20689_SENSOR_ACCELEROMETER))*/)
		data_output_control1_mask |= ACCEL_SET;
	
	if( (s->enabled_sensor_mask & (1<<INV_icm20689_SENSOR_RAW_GYROSCOPE)) ||
	    (s->enabled_sensor_mask & (1<<INV_icm20689_SENSOR_GYROSCOPE)) ||
		(s->enabled_sensor_mask & (1<<INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED)))
		data_output_control1_mask |= GYRO_SET;
		
	if( s->enabled_sensor_mask & (1<<INV_icm20689_SENSOR_GAME_ROTATION_VECTOR) ) {
		data_output_control1_mask |= QUAT6_SET;
		data_output_control1_mask |= GYRO_SET;
	}
	
	result |= dmp_icm20689_set_data_output_control1(s, data_output_control1_mask);

	// enable HW sensors
	if (s->enabled_sensor_mask & INV_NEEDS_GYRO_MASK)
		data_rdy_status |= GYRO_AVAILABLE;
	if (s->enabled_sensor_mask & INV_NEEDS_ACCEL_MASK)
		data_rdy_status |= ACCEL_AVAILABLE;
	if (s->enabled_sensor_mask & INV_NEEDS_TEMP_MASK)
		data_rdy_status |= TEMP_AVAILABLE;

    // turn on gyro cal only if gyro is available
    if (data_rdy_status & GYRO_AVAILABLE)
	    motion_event_control_mask |= INV_GYRO_CAL_EN;
	result |= dmp_icm20689_set_motion_event_control(s, motion_event_control_mask);

	result |= set_hw_smplrt_dmp_odrs(s, &resulting_divider);
	result |= inv_icm20689_enable_mems(s, (int)data_rdy_status, resulting_divider);

	// need to reset and restart FIFO with new configurations
	result |= inv_icm20689_enable_fifo(s);
	result |= inv_icm20689_enable_dmp(s);

	return result;
}
#if 0
int inv_icm20689_configure_accel_wom(struct inv_icm20689 * s, uint8_t wom_threshold)
{
	int result = 0;
	uint8_t data;

	if(wom_threshold) {
		// Set threshold
		// wom_threshold = 0x0F;
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_WOM_THR,   1, &wom_threshold);
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_WOM_X_THR, 1, &wom_threshold);
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_WOM_Y_THR, 1, &wom_threshold);
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_WOM_Z_THR, 1, &wom_threshold);

		// Enable wake on motion interrupt on all 3 axes
		result |= inv_icm20689_mems_read_reg(s, MPUREG_INT_ENABLE, 1, &data);
		data |= 0xE0;
		data &= ~BIT_DATA_RDY_INT_EN; // clear data ready interrupt
		result |= inv_icm20689_mems_write_reg(s, MPUREG_INT_ENABLE, 1, &data);

		// Enable WOM logic and set WOM interrupt mode
		// bit 7: Enable the WOM logic
		// bit 6: 1: compare the current sample with the previous sample.
		//        0: initial sample is stored, all future samples are compared to the initial sample.
		// bit 0: 0 – Set WoM interrupt on OR of x,y or z axis interrupt
		//        1 – Set WoM interrupt on AND of x, y and z axes interrupts
		result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);
		data |= 0xC0; // Compare current sample with the previous sample.
		              // Set WoM interrupt on OR of xyz axes interrupt.
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);

		// Wake up chip and set to LP mode
		result |= inv_icm20689_mems_read_reg(s, MPUREG_PWR_MGMT_1, 1, &data);
		data &= ~BIT_SLEEP; // Clear sleep bit
		data |= BIT_CYCLE; // Enable accel cycling mode.
		result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &data);

		// Enable accel
		result |= inv_icm20689_mems_read_reg(s, MPUREG_PWR_MGMT_2, 1, &data);
		data &= ~BIT_PWR_ACCEL_STBY; // Enable accel
		data |= BIT_PWR_GYRO_STBY; // Disable gyro.
		result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &data);

		s->wom_enabled = 1;
	}
	else {
		uint16_t resulting_divider = 0;
		uint16_t data_rdy_status = 0;

		// Disable wake on motion interrupt on all 3 axes
		result |= inv_icm20689_mems_read_reg(s, MPUREG_INT_ENABLE, 1, &data);
		data &= ~0xE0;
		data |= BIT_DATA_RDY_INT_EN; // Set data ready interrupt
		result |= inv_icm20689_mems_write_reg(s, MPUREG_INT_ENABLE, 1, &data);

		// Disable WOM logic and set WOM interrupt mode
		result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);
		data &= ~0x80; // Disable WoM logic
		result |= inv_icm20689_mems_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);

		// Disable accel
		result |= inv_icm20689_mems_read_reg(s, MPUREG_PWR_MGMT_2, 1, &data);
		data |= BIT_PWR_ACCEL_STBY; // Disable accel
		result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &data);

		// Put chip to sleep
		result |= inv_icm20689_mems_read_reg(s, MPUREG_PWR_MGMT_1, 1, &data);
		data |= BIT_SLEEP; // Set sleep bit
		result |= inv_icm20689_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &data);
		s->power_state = PowerStateSleepState;

		// Re-enable sensors with data ready interrupt configuration
		if (s->android_sensor_mask & INV_NEEDS_GYRO_MASK) 
			data_rdy_status |= GYRO_AVAILABLE;
		if (s->android_sensor_mask & INV_NEEDS_ACCEL_MASK)
			data_rdy_status |= ACCEL_AVAILABLE;
		if (s->android_sensor_mask & INV_NEEDS_TEMP_MASK)
			data_rdy_status |= TEMP_AVAILABLE;

		result |= inv_icm20689_disable_fifo(s);
		result |= set_hw_smplrt_dmp_odrs(s, &resulting_divider);
		result |= inv_icm20689_enable_mems(s, (int)data_rdy_status, resulting_divider);
		// need to reset and restart FIFO with data ready interrupt configuration
		result |= inv_icm20689_enable_fifo(s, (int)data_rdy_status);

		s->wom_enabled = 0;
	}

	return result;
}
#endif

int inv_icm20689_disable_dmp(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t reg;

	/* disable DMP */
	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg &= ~BIT_DMP_EN;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);
	inv_icm20689_sleep(5);

	return result;
}

int inv_icm20689_disable_fifo(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t reg;

	/* disable FIFO */
	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg &= ~BIT_FIFO_EN;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
}

int inv_icm20689_enable_dmp(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t reg;

	/* enable DMP */
	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg |= BIT_DMP_EN;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
}

int inv_icm20689_enable_fifo(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t reg;

	/* enable FIFO */
	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg |= BIT_FIFO_EN;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
}

int inv_icm20689_reset_dmp(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t reg;

	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg |= BIT_DMP_RST;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
}

int inv_icm20689_reset_fifo(struct inv_icm20689 * s)
{
	int result = 0;
	uint8_t reg;

	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg |= BIT_FIFO_RST;
	result |= inv_icm20689_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
}

// get if all sensors are off
int inv_icm20689_all_sensors_off(struct inv_icm20689 * s)
{
	if( (s->enabled_sensor_mask == 0) && (s->wom_enabled == 0) )
		return 1;
	return 0;
}

int inv_icm20689_poll_sensor(struct inv_icm20689 * s, void * context,
		void (*handler)(void * context, enum inv_icm20689_sensor sensor, uint64_t timestamp, const void * data, const void *arg))
{
	unsigned short header=0, header2 = 0; 
	int data_left_in_fifo=0;
	short short_data[3] = {0};
	signed long  long_data[3] = {0};
	signed long  long_quat[3] = {0};
	float gyro_raw_float[3];
	float gyro_bias_float[3];
	static int gyro_accuracy = 0;
	int dummy_accuracy = 0;

	float grv_float[4];
	float gyro_float[3];
	static uint64_t currentIrqTimeUs = 0;
	unsigned short sample_cnt_array[GENERAL_SENSORS_MAX] = { 0 };
	long long ts = 0;
	
	unsigned short total_sample_cnt = 0;
	currentIrqTimeUs = inv_icm20689_get_time_us();
	do {

		/* Mirror FIFO contents and stop processing FIFO if an error was detected*/
		if (inv_icm20689_fifo_swmirror(s, &data_left_in_fifo, &total_sample_cnt, sample_cnt_array))
			break;
		ts = currentIrqTimeUs;
			
		while(total_sample_cnt--) {
			/* Read FIFO contents and parse it, and stop processing FIFO if an error was detected*/
			if (inv_icm20689_fifo_pop(s, &header, &header2, &data_left_in_fifo))
				break;
				
			/* Gyro sample available from DMP FIFO */
			if (header & GYRO_SET) {
				float lScaleDeg = (1 << inv_icm20689_get_gyro_fullscale(s)) * 250.f; // From raw to dps to degree per seconds
				float lScaleDeg_bias = 2000.f; // Gyro bias from FIFO is always in 2^20 = 2000 dps regardless of fullscale 
				signed long  raw_data[3] = {0};
				signed long  bias_data[3] = {0};

				/* Read raw gyro out of DMP FIFO and convert it from Q15 raw data format to degrees per seconds */
				inv_icm20689_dmp_get_raw_gyro(short_data);  
				raw_data[0] = (long) short_data[0];
				raw_data[1] = (long) short_data[1];
				raw_data[2] = (long) short_data[2];
				inv_icm20689_convert_dmp3_to_body(s, raw_data, lScaleDeg/(1L<<15), gyro_raw_float);                            
					
				if(inv_icm20689_is_sensor_enabled(s, INV_icm20689_SENSOR_RAW_GYROSCOPE)) {
					long out[3];
					inv_icm20689_convert_quat_rotate_fxp(s->s_quat_chip_to_body, raw_data, out);
					//s->timestamp[INV_ICM20648_SENSOR_RAW_GYROSCOPE] += s->sensorlist[INV_ICM20648_SENSOR_RAW_GYROSCOPE].odr_applied_us;
					handler(context, INV_icm20689_SENSOR_RAW_GYROSCOPE, ts, out, &dummy_accuracy);
				}
					
				/* Read bias gyro out of DMP FIFO and convert it from raw data format to degrees per seconds */
				inv_icm20689_dmp_get_gyro_bias(bias_data);
				signed long scaled_bias[3];
				scaled_bias[0] = bias_data[0]/2859;
				scaled_bias[1] = bias_data[1]/2859;
				scaled_bias[2] = bias_data[2]/2859;

				inv_icm20689_convert_dmp3_to_body(s, scaled_bias, lScaleDeg_bias/(1L<<15), gyro_bias_float);
					
				if ((bias_data[0] != 0) && (bias_data[1] != 0) && (bias_data[2] != 0))
					gyro_accuracy = 3;
				else
					gyro_accuracy = 0;
					
				if(inv_icm20689_is_sensor_enabled(s, INV_icm20689_SENSOR_GYROSCOPE)) {
					/* Compute calibrated gyro data based on raw and bias gyro data and convert it from Q20 raw data format to radian per seconds in Android format */
					inv_mems_dmp_get_calibrated_gyro(long_data, short_data, bias_data);
					inv_icm20689_convert_dmp3_to_body(s, long_data, lScaleDeg_bias/(1L<<30), gyro_float);
					handler(context, INV_icm20689_SENSOR_GYROSCOPE, ts, gyro_float, &gyro_accuracy);
				}
				
				if(inv_icm20689_is_sensor_enabled(s, INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED)) {
					float raw_bias_gyr[6];
					raw_bias_gyr[0] = gyro_raw_float[0];
					raw_bias_gyr[1] = gyro_raw_float[1];
					raw_bias_gyr[2] = gyro_raw_float[2];
					raw_bias_gyr[3] = gyro_bias_float[0];
					raw_bias_gyr[4] = gyro_bias_float[1];
					raw_bias_gyr[5] = gyro_bias_float[2];
					/* send raw float and bias for uncal gyr*/
					handler(context, INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED, ts, raw_bias_gyr, &gyro_accuracy);
				}
			}
			/* accel sample available from DMP FIFO */
			if (header & ACCEL_SET) {
				/* Read calibrated accel out of DMP FIFO and convert it from Q25 raw data format to m/s² in Android format */
				inv_icm20689_dmp_get_accel(long_data);

				if(inv_icm20689_is_sensor_enabled(s, INV_icm20689_SENSOR_RAW_ACCELEROMETER)) {
					long out[3];
					inv_icm20689_convert_quat_rotate_fxp(s->s_quat_chip_to_body, long_data, out);
					out[0] /= 1<<15;
					out[1] /= 1<<15;
					out[2] /= 1<<15;
					//s->timestamp[INV_ICM20648_SENSOR_RAW_GYROSCOPE] += s->sensorlist[INV_ICM20648_SENSOR_RAW_GYROSCOPE].odr_applied_us;
					handler(context, INV_icm20689_SENSOR_RAW_ACCELEROMETER, ts, out, &dummy_accuracy);
				}
/*
				if(inv_icm20689_is_sensor_enabled(s, INV_icm20689_SENSOR_ACCELEROMETER)) {
					float scale;
					accel_accuracy = inv_icm20689_get_accel_accuracy();
					scale = (1 << inv_icm20689_get_accel_fullscale(s)) * 2.f / (1L<<30); // Convert from raw units to g's

					inv_icm20689_convert_dmp3_to_body(s, long_data, scale, accel_float);

					handler(context, INV_icm20689_SENSOR_ACCELEROMETER, ts, accel_float, &accel_accuracy);
				}
*/				
			}
			/* 6axis AG orientation quaternion sample available from DMP FIFO */
			if (header & QUAT6_SET) {
				float ref_quat[4];
				/* Read 6 axis quaternion out of DMP FIFO in Q30 */
				inv_icm20689_dmp_get_6quaternion(long_quat);
				if(inv_icm20689_is_sensor_enabled(s, INV_icm20689_SENSOR_GAME_ROTATION_VECTOR)) {
					/* and convert it from Q30 DMP format to Android format only if GRV sensor is enabled */
					inv_icm20689_convert_rotation_vector(s, long_quat, grv_float);
					ref_quat[0] = grv_float[3];
					ref_quat[1] = grv_float[0];
					ref_quat[2] = grv_float[1];
					ref_quat[3] = grv_float[2];
					handler(context, INV_icm20689_SENSOR_GAME_ROTATION_VECTOR, ts, ref_quat, &gyro_accuracy);
				}					
			}
		}
	} while(data_left_in_fifo);
		
	return 0;
}
