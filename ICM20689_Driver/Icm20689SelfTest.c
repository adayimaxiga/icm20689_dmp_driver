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

#include "icm20689SelfTest.h"
#include "icm20689Defs.h"
#include "icm20689.h"

#define DEF_ST_PRECISION				1000
#define DEF_ST_ACCEL_FS_MG				2000
#define DEF_ST_TRY_TIMES				2
#define DEF_ST_SAMPLES					200
#define DEF_GYRO_CT_SHIFT_DELTA			500
#define DEF_ACCEL_ST_SHIFT_DELTA		500
#define ACCEL_ST_AL_MIN ((DEF_ACCEL_ST_AL_MIN * DEF_ST_SCALE \
				 / DEF_ST_ACCEL_FS_MG) * DEF_ST_PRECISION)
#define ACCEL_ST_AL_MAX ((DEF_ACCEL_ST_AL_MAX * DEF_ST_SCALE \
				 / DEF_ST_ACCEL_FS_MG) * DEF_ST_PRECISION)
#define DEF_ST_ACCEL_LPF				2
#define DEF_ST_GYRO_LPF					2
#define DEF_SELFTEST_SAMPLE_RATE		0 /* 1000Hz */
#define DEF_SELFTEST_SAMPLE_RATE_LP		4 /*  200Hz */
#define DEF_SELFTEST_ACCEL_FS			(0 << 3) /* 2g */
#define DEF_SELFTEST_GYRO_FS			(0 << 3) /* 250dps */
#define DEF_ST_STABLE_TIME				20
#if ((MEMS_CHIP != HW_ICM20602)&&(MEMS_CHIP != HW_ICM20603))
#define DEF_GYRO_WAIT_TIME				30
#define DEF_GYRO_WAIT_TIME_LP			150
#else
#define DEF_GYRO_WAIT_TIME				5
#define DEF_GYRO_WAIT_TIME_LP			50
#endif

/* Gyro Offset Max Value (dps) */
#define DEF_GYRO_OFFSET_MAX				20
/* Gyro Self Test Absolute Limits ST_AL (dps) */
#define DEF_GYRO_ST_AL					60
/* Accel Self Test Absolute Limits ST_AL (mg) */
#define DEF_ACCEL_ST_AL_MIN				225
#define DEF_ACCEL_ST_AL_MAX				675

enum {
	ST_NORMAL_MODE = 0,
	ST_GYRO_LP_MODE = 1,
	ST_ACCEL_LP_MODE = 2
};

struct recover_regs {
	uint8_t int_enable;		/* REG_INT_ENABLE 		*/
	uint8_t fifo_en;		/* REG_FIFO_EN 			*/
	uint8_t user_ctrl;		/* REG_USER_CTRL 		*/
	uint8_t config;			/* REG_CONFIG 			*/
	uint8_t gyro_config;	/* REG_GYRO_CONFIG 		*/
	uint8_t accel_config;	/* REG_ACCEL_CONFIG		*/
	uint8_t accel_config_2;	/* REG_ACCEL_CONFIG_2	*/
	uint8_t smplrt_div;		/* REG_SAMPLE_RATE_DIV	*/
	uint8_t lp_mode;		/* REG_LP_MODE_CTRL		*/
	uint8_t pwr_mgmt_1;		/* REG_PWR_MGMT_1		*/
	uint8_t pwr_mgmt_2;		/* REG_PWR_MGMT_2		*/
};

#if (MEMS_CHIP != HW_ICM20609)
static const uint16_t sSelfTestEquation[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};
#endif

#if (MEMS_CHIP != HW_ICM20609)
static int save_settings(struct inv_icm20689 * s, struct recover_regs * saved_regs)
{
	int result = 0;

	result |= inv_icm20689_mems_read_reg(s, MPUREG_PWR_MGMT_1, 1, &saved_regs->pwr_mgmt_1);

	/* wake up */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_1, CLK_SEL);

	result |= inv_icm20689_mems_read_reg(s, MPUREG_INT_ENABLE, 1, &saved_regs->int_enable);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_FIFO_EN, 1, &saved_regs->fifo_en);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_USER_CTRL, 1, &saved_regs->user_ctrl);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_CONFIG, 1, &saved_regs->config);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_GYRO_CONFIG, 1, &saved_regs->gyro_config);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_CONFIG, 1, &saved_regs->accel_config);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &saved_regs->accel_config_2);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_SMPLRT_DIV, 1, &saved_regs->smplrt_div);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_LP_CONFIG, 1, &saved_regs->lp_mode);
	result |= inv_icm20689_mems_read_reg(s, MPUREG_PWR_MGMT_2, 1, &saved_regs->pwr_mgmt_2);

	return result;
}
#endif
#if (MEMS_CHIP != HW_ICM20609)

static int recover_settings(struct inv_icm20689 * s, const struct recover_regs * saved_regs)
{
	int result = 0;

	// Stop sensors
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_2, BIT_PWR_ALL_OFF);

	// Restore sensor configurations
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_INT_ENABLE, saved_regs->int_enable);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_FIFO_EN, saved_regs->fifo_en);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_USER_CTRL, saved_regs->user_ctrl);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_CONFIG, saved_regs->config);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_GYRO_CONFIG, saved_regs->gyro_config);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_ACCEL_CONFIG, saved_regs->accel_config);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_ACCEL_CONFIG_2, saved_regs->accel_config_2);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_SMPLRT_DIV, saved_regs->smplrt_div);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_LP_CONFIG, saved_regs->lp_mode);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_1, saved_regs->pwr_mgmt_1);
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_2, saved_regs->pwr_mgmt_2);

	return result;
}
#endif

#if (MEMS_CHIP != HW_ICM20609)
/**
*  @brief check gyro self test
*  @param[in] meanNormalTestValues average value of normal test.
*  @param[in] meanSelfTestValues   average value of self test
*  @return zero as success. A non-zero return value indicates failure in self test.
*/
static int check_gyro_self_test(struct inv_icm20689 * s, int *meanNormalTestValues, int *meanSelfTestValues)
{
	int ret_val = 0;

#if ((MEMS_CHIP != HW_ICM20690) && (MEMS_CHIP != HW_ICM20602) && (MEMS_CHIP != HW_ICM20603) && (MEMS_CHIP != HW_icm20689))

	(void)s, (void)meanNormalTestValues, (void)meanSelfTestValues;

	return ret_val;
#else
	uint8_t regs[3];
	int otp_value_zero = 0;
	int st_shift_prod[3], st_shift_cust[3], i;
	int result;

	result = inv_icm20689_mems_read_reg(s, MPUREG_SELF_TEST_X_GYRO, 3, regs);
	if (result)
		return result;

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}

	for (i = 0; i < 3; i++) {
		st_shift_cust[i] = meanSelfTestValues[i] - meanNormalTestValues[i];
		if (!otp_value_zero) {
			/* Self Test Pass/Fail Criteria A */
			if (st_shift_cust[i] < DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i])
				ret_val = 1;
		} else {
			/* Self Test Pass/Fail Criteria B */
			if (st_shift_cust[i] < DEF_GYRO_ST_AL * DEF_SELFTEST_GYRO_SENS * DEF_ST_PRECISION)
				ret_val = 1;
		}
	}

	if (ret_val == 0) {
		/* Self Test Pass/Fail Criteria C */
		for (i = 0; i < 3; i++) {
			if (INV_ABS(meanNormalTestValues[i]) > DEF_GYRO_OFFSET_MAX * DEF_SELFTEST_GYRO_SENS * DEF_ST_PRECISION)
				ret_val = 1;
		}
	}

	return ret_val;
#endif
}
#endif

#if (MEMS_CHIP != HW_ICM20609)
/**
*  @brief check accel self test
*  @param[in] meanNormalTestValues average value of normal test.
*  @param[in] meanSelfTestValues   average value of self test
*  @return zero as success. A non-zero return value indicates failure in self test.
*/
static int check_accel_self_test(struct inv_icm20689 * s, int *meanNormalTestValues, int *meanSelfTestValues)
{
	int ret_val = 0;

#if ((MEMS_CHIP != HW_ICM20690) && (MEMS_CHIP != HW_ICM20602) && (MEMS_CHIP != HW_ICM20603) && (MEMS_CHIP != HW_icm20689))

	(void)s, (void)meanNormalTestValues, (void)meanSelfTestValues;

	return ret_val;
#else
	uint8_t regs[3];
	int otp_value_zero = 0;
	int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
	int result;

	result = inv_icm20689_mems_read_reg(s, MPUREG_SELF_TEST_X_ACCEL, 3, regs);
	if (result)
		return result;

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}

	if (!otp_value_zero) {
		/* Self Test Pass/Fail Criteria A */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = meanSelfTestValues[i] - meanNormalTestValues[i];
			st_shift_ratio[i] = INV_ABS(st_shift_cust[i] / st_shift_prod[i] - DEF_ST_PRECISION);
			if (st_shift_ratio[i] > DEF_ACCEL_ST_SHIFT_DELTA)
				ret_val = 1;
		}
	} else {
		/* Self Test Pass/Fail Criteria B */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = INV_ABS(meanSelfTestValues[i] - meanNormalTestValues[i]);
			if (st_shift_cust[i] < ACCEL_ST_AL_MIN || st_shift_cust[i] > ACCEL_ST_AL_MAX)
				ret_val = 1;
		}
	}

	return ret_val;
#endif
}
#endif

#if (MEMS_CHIP != HW_ICM20609)
/*
*  do_test() - do the actual test of self testing
*/
static int do_test(struct inv_icm20689 * s, int self_test_flag, int *gyro_result, int *accel_result, int lp_mode)
{
	int result = 0;

#if ((MEMS_CHIP != HW_ICM20690) && (MEMS_CHIP != HW_ICM20602) && (MEMS_CHIP != HW_ICM20603) && (MEMS_CHIP != HW_icm20689))

	(void)s, (void)self_test_flag, (void)gyro_result, (void)accel_result, (void)lp_mode;

    return result;
#else
	int i, j, packet_size;
	uint8_t data[BYTES_PER_SENSOR * 2 + BYTES_PER_TEMP_SENSOR], d, dd;
	int fifo_count, packet_count, ind, ss;

    /* switch engine */
	d = 0;
	if (lp_mode == ST_GYRO_LP_MODE)
		d = BIT_PWR_ACCEL_STBY;
	else if (lp_mode == ST_ACCEL_LP_MODE)
		d = BIT_PWR_GYRO_STBY;
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_2, d);
	inv_icm20689_sleep(GYRO_ENGINE_UP_TIME);

	packet_size = BYTES_PER_SENSOR * 2;

#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
	packet_size += BYTES_PER_TEMP_SENSOR;
#endif

	/* clear signal path */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_USER_CTRL, 1);

	inv_icm20689_sleep(30);

	/* disable interrupt */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_INT_ENABLE, 0);

	/* disable the sensor output to FIFO */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_FIFO_EN, 0);

	/* disable fifo reading */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_USER_CTRL, 0);

	/* setup parameters */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_CONFIG, DEF_ST_GYRO_LPF);

	/* gyro lp mode */
	if (lp_mode == ST_GYRO_LP_MODE)
		d  = BIT_GYRO_CYCLE;
	else
		d = 0;
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_LP_CONFIG, d);

	/* config accel LPF register */
#if ((MEMS_CHIP == HW_ICM20690) || (MEMS_CHIP == HW_icm20689))
	d = BITS_FIFO_SIZE; 
#else
	d = 0; /* 1kB by default, 20602 */
#endif
	if (lp_mode == ST_ACCEL_LP_MODE)
		d |= BITS_ACCEL_FCHOICE_B;
	else
		d |= DEF_ST_ACCEL_LPF;
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_ACCEL_CONFIG_2, d);

	if (lp_mode)
		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_SMPLRT_DIV, DEF_SELFTEST_SAMPLE_RATE_LP);
	else
		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_SMPLRT_DIV, DEF_SELFTEST_SAMPLE_RATE);

	/* wait for the sampling rate change to stabilize */
	inv_icm20689_sleep(GYRO_ENGINE_UP_TIME);

	d = self_test_flag | DEF_SELFTEST_GYRO_FS;
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_GYRO_CONFIG, d);

	d = self_test_flag | DEF_SELFTEST_ACCEL_FS;
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_ACCEL_CONFIG, d);

	/* wait for the output to get stable */
	inv_icm20689_sleep(DEF_ST_STABLE_TIME);

	/* enable sensor output to FIFO */
	d = BIT_GYRO_OUT | BIT_ACCEL_OUT;
	for (i = 0; i < THREE_AXES; i++) {
		gyro_result[i] = 0;
		accel_result[i] = 0;
	}

	ss = 0;
	while(ss < 200) {
		/* clear FIFO */
		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_USER_CTRL, BIT_FIFO_RST);

		/* enable FIFO reading */
		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_USER_CTRL, BIT_FIFO_EN);

		/* accel lp mode */
		dd = CLK_SEL;
		if (lp_mode == ST_ACCEL_LP_MODE)
			dd |= BIT_CYCLE;

		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_FIFO_EN, d);

		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_1, dd);

		if (lp_mode)
			inv_icm20689_sleep(DEF_GYRO_WAIT_TIME_LP);
		else
			inv_icm20689_sleep(DEF_GYRO_WAIT_TIME);

		/* accel lp mode */
		if (lp_mode == ST_ACCEL_LP_MODE)
			result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_1, CLK_SEL);

#if ((MEMS_CHIP != HW_ICM20602) && (MEMS_CHIP != HW_ICM20603))
		result |= inv_icm20689_mems_write_reg_one(s, MPUREG_FIFO_EN, 0); // stop FIFO
#endif
		result |= inv_icm20689_mems_read_reg(s, MPUREG_FIFO_COUNTH, FIFO_COUNT_BYTE, data);

		fifo_count = (data[0] << 8) | data[1];
		packet_count = fifo_count / packet_size;

		i = 0;
		while ((i < packet_count) && (ss < 200)) {
			short vals[3];
			result |= inv_icm20689_mems_read_reg(s, MPUREG_FIFO_R_W, packet_size, data);
			ind = 0;
			for (j = 0; j < THREE_AXES; j++) {
				vals[j] = (data[ind + 2 * j] << 8) | data[ind + 2 * j + 1];
				accel_result[j] += vals[j];
			}
			ind += BYTES_PER_SENSOR;
#if ((MEMS_CHIP == HW_ICM20602) || (MEMS_CHIP == HW_ICM20603))
			ind += BYTES_PER_TEMP_SENSOR;
#endif
			for (j = 0; j < THREE_AXES; j++) {
				vals[j] = (data[ind + 2 * j] << 8) | data[ind + 2 * j + 1];
				gyro_result[j] += vals[j];
			}
			ss++;
			i++;
		}
	}

	for (j = 0; j < THREE_AXES; j++) {
		accel_result[j] = accel_result[j] / ss;
		accel_result[j] *= DEF_ST_PRECISION;
	}
	for (j = 0; j < THREE_AXES; j++) {
		gyro_result[j] = gyro_result[j] / ss;
		gyro_result[j] *= DEF_ST_PRECISION;
	}

	return result;
#endif
}
#endif

int inv_icm20689_run_selftest(struct inv_icm20689 * s)
{
#if ((MEMS_CHIP != HW_ICM20690) && (MEMS_CHIP != HW_ICM20602) && (MEMS_CHIP != HW_ICM20603) && (MEMS_CHIP != HW_icm20689))

	(void)s;

	return -1;
#else
	int result = 0;
	int gyro_bias_st[THREE_AXES], gyro_bias_regular[THREE_AXES];
	int accel_bias_st[THREE_AXES], accel_bias_regular[THREE_AXES];
	int gyro_bias_regular_lp[THREE_AXES], accel_bias_regular_lp[THREE_AXES];
	int test_times;
	char accel_result, gyro_result, compass_result;
	int i;
	struct recover_regs recover_regs;

	result |= save_settings(s, &recover_regs);

	/* enable sensors */
	result |= inv_icm20689_mems_write_reg_one(s, MPUREG_PWR_MGMT_2, 0);
	inv_icm20689_sleep(GYRO_ENGINE_UP_TIME);

	accel_result = 0;
	gyro_result = 0;
	compass_result = 0;

	/* (1) hardware self-test */
	/* Collect averaged data of gyro and accel in normal mode.
	   Try up to DEF_ST_TRY_TIMES times in case of some error such as I2C error.
	   The collected data can be utilized as factory bias of normal mode.
	   Will be checked later.*/
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = do_test(s, 0, gyro_bias_regular, accel_bias_regular, ST_NORMAL_MODE);
		if (result)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;

	for (i = 0; i < 3; i++) {
		s->gyro_ois_st_bias[i] = gyro_bias_regular[i] / DEF_ST_PRECISION;
		s->accel_ois_st_bias[i] = accel_bias_regular[i] / DEF_ST_PRECISION;
	}

	/* (2) hardware self-test */
	/* Collect averaged data of gyro and accel in hardware selftest mode.
	   Try up to DEF_ST_TRY_TIMES times in case of some error such as I2C error.
	   Will be checked later */	
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = do_test(s, (BIT_XG_ST | BIT_YG_ST | BIT_ZG_ST), gyro_bias_st, accel_bias_st, ST_NORMAL_MODE);
		if (result)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;

	/* (3) Bias collection for accel lp mode */
	/* Collect averaged accel data in lp mode.
	   Try up to DEF_ST_TRY_TIMES times in case of some error such as I2C error.
	   This is only for factory bias of accel lp mode */	
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = do_test(s, 0, gyro_bias_regular_lp, accel_bias_regular_lp, ST_ACCEL_LP_MODE);
		if (result)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;

	for (i = 0; i < 3; i++) {
		s->accel_st_bias[i] = accel_bias_regular_lp[i] / DEF_ST_PRECISION;
	}

	/* (4) Bias collection for gyro lp mode */
	/* Collect averaged gyro data in lp mode.
	   Try up to DEF_ST_TRY_TIMES times in case of some error such as I2C error.
	   This is only for factory bias of gyro lp mode */	
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = do_test(s, 0, gyro_bias_regular_lp, accel_bias_regular_lp, ST_GYRO_LP_MODE);
		if (result)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;

	for (i = 0; i < 3; i++) {
		s->gyro_st_bias[i] = gyro_bias_regular_lp[i] / DEF_ST_PRECISION;
	}

	/* Check data collected in step (1) and (2).
	   Compare with OTP values which are stored at factory */
	accel_result = !check_accel_self_test(s, accel_bias_regular, accel_bias_st);
	gyro_result = !check_gyro_self_test(s, gyro_bias_regular, gyro_bias_st);

#if (MEMS_CHIP == HW_ICM20690)
	compass_result = !inv_icm20689_check_akm_self_test(s);
#endif
test_fail:
	recover_settings(s, &recover_regs);

	/* The accelerometer bias computed by selftest must not be directly apply to offset registers.
       We have to compute specific values based on OTP registers before applying the values to offset registers */
	s->accel_reg_bias_computed = 0;

	return (compass_result << 2) | (accel_result << 1) | gyro_result;

#endif
}

void inv_icm20689_get_st_bias(struct inv_icm20689 * s, int * st_bias)
{
	int axis, axis_sign;
	int gravity;
	int i, t;
	int check;
	int scale;

	/* check bias there ? */
	check = 0;
	for (i = 0; i < 3; i++) {
		if (s->gyro_ois_st_bias[i] != 0)
			check = 1;
		if (s->gyro_st_bias[i] != 0)
			check = 1;
		if (s->accel_ois_st_bias[i] != 0)
			check = 1;
		if (s->accel_st_bias[i] != 0)
			check = 1;
	}

	/* if no bias, return all 0 */
	if (check == 0) {
		for (i = 0; i < 12; i++)
			st_bias[i] = 0;
		return;
	}

	/* dps scaled by 2^16 */
	scale = 65536 / DEF_SELFTEST_GYRO_SENS;

	/* Gyro normal mode */
	t = 0;
	for (i = 0; i < 3; i++)
		st_bias[i + t] = s->gyro_ois_st_bias[i] * scale;

	/* Gyro LP mode */
	t += 3;
	for (i = 0; i < 3; i++)
		st_bias[i + t] = s->gyro_st_bias[i] * scale;

	axis = 0;
	axis_sign = 1;
	if (INV_ABS(s->accel_ois_st_bias[1]) > INV_ABS(s->accel_ois_st_bias[0]))
		axis = 1;
	if (INV_ABS(s->accel_ois_st_bias[2]) > INV_ABS(s->accel_ois_st_bias[axis]))
		axis = 2;
	if (s->accel_ois_st_bias[axis] < 0)
		axis_sign = -1;

	/* gee scaled by 2^16 */
	scale = 65536 / (DEF_ST_SCALE / (DEF_ST_ACCEL_FS_MG / 1000));

	gravity = 32768 / (DEF_ST_ACCEL_FS_MG / 1000) * axis_sign;
	gravity *= scale;

	/* Accel normal mode */
	t += 3;
	for (i = 0; i < 3; i++) {
		st_bias[i + t] = s->accel_ois_st_bias[i] * scale;
		if (axis == i)
			st_bias[i + t] -= gravity;
	}

	/* Accel LP mode */
	t += 3;
	for (i = 0; i < 3; i++) {
		st_bias[i + t] = s->accel_st_bias[i] * scale;
		if (axis == i)
			st_bias[i + t] -= gravity;
	}
}

void inv_icm20689_set_st_bias(struct inv_icm20689 * s, int * st_bias)
{
	int i, t;
	int scale;
	int check_gyro = 0;
	int check_acc = 0;

	/* check bias there ? */
	for (i = 0; i < 6; i++) {
		if (st_bias[i] != 0)
			check_gyro = 1;
		if (st_bias[i + 6] != 0)
			check_acc = 1;
	}

	if((check_gyro == 0) && (check_acc == 0))
		return;

	if(check_gyro) {
		/* dps scaled by 2^16 */
		scale = 65536 / DEF_SELFTEST_GYRO_SENS;

		/* Gyro normal mode */
		t = 0;
		for (i = 0; i < 3; i++)
			s->gyro_ois_st_bias[i] = st_bias[i + t] / scale;

		/* Gyro LP mode */
		t += 3;
		for (i = 0; i < 3; i++)
			s->gyro_st_bias[i] = st_bias[i + t] / scale;
	}
	
	if(check_acc) {
		int gravity;
		int axis = 0;
		int axis_sign = 1;
		if (INV_ABS(st_bias[1 + 6]) > INV_ABS(st_bias[0 + 6]))
			axis = 1;
		if (INV_ABS(st_bias[2 + 6]) > INV_ABS(st_bias[axis + 6]))
			axis = 2;
		if (st_bias[axis + 6] < 0)
			axis_sign = -1;

		/* gee scaled by 2^16 */
		scale = 65536 / (DEF_ST_SCALE / (DEF_ST_ACCEL_FS_MG / 1000));

		gravity = 32768 / (DEF_ST_ACCEL_FS_MG / 1000) * axis_sign;

		/* Accel normal mode */
		t = 6;
		for (i = 0; i < 3; i++) {
			s->accel_ois_st_bias[i] = st_bias[i + t] / scale;
			if (axis == i)
				s->accel_ois_st_bias[i] += gravity;
		}

		/* Accel LP mode */
		t += 3;
		for (i = 0; i < 3; i++) {
			s->accel_st_bias[i] = st_bias[i + t] / scale;
			if (axis == i)
				s->accel_st_bias[i] += gravity;
		}
	}
}
