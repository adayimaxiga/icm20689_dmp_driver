/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
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

/* InvenSense drivers and utils */
#include "sys.h"
#include "icm20689.h"
#include "icm20689Defs.h"
#include "icm20689MPUFifoControl.h"
#include "SensorTypes.h"
/*****************************************************************/
#include "sensor.h"
#include <math.h>

static const uint8_t dmp3_image[] = {
	#include "icm20689_img.dmp3.h"
};

static const uint8_t EXPECTED_WHOAMI[] = {0x98};  /* WHOAMI value for ICM20689 */

/* FSR configurations */
int32_t cfg_acc_fsr = 4000; // Accel FSR must be set as +/- 4g and cannot be changed. 
int32_t cfg_gyr_fsr = 2000; // Gyro FSR must be set at +/- 2000dps and cannot be changed.

inv_icm20689_t icm_device;

/*
 * Mask to keep track of enabled sensors
 */
uint32_t user_enabled_sensor_mask = 0;

/* Forward declaration */
static void icm20689_apply_mounting_matrix(void);
static int icm20689_run_selftest(void);
static void check_rc(int rc, const char * context_str);
static void handle_data_function(void * context, uint8_t sensortype, uint64_t timestamp, const void * data, const void *arg);

//void sensor_event(const inv_sensor_event_t * event, void * arg);

/*
 * Variable to keep track of the expected period common for all sensors
 */
uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

/*
 * Variable to drop the first timestamp(s) after a sensor start cached by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
//static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
static uint8_t chip_info[3];

/* 
 * Mounting matrix configuration applied for both Accel and Gyro 
 * The coefficient values are coded in integer q30
 */
static float cfg_mounting_matrix[9]= {1.f,   0,   0,
										0, 1.f,   0,
										0,   0, 1.f };

static uint8_t convert_to_generic_ids[INV_icm20689_SENSOR_MAX] = {
	/*INV_SENSOR_TYPE_ACCELEROMETER,*/
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_CUSTOM_PRESSURE };

uint64_t inv_icm20689_get_time_us(void) {
	return 0;
}

/******************************************************************************************/
/*
 * Sleep implementation for icm20689
 */
void inv_icm20689_sleep(int ms)
{
	HAL_Delay(ms);
}

void inv_icm20689_sleep_us(int us)
{
	HAL_Delay(us);
}

/******************************************************************************************/
//³õÊ¼»¯º¯Êý£¬×îÖØÒªµÄÒ»¸ö
int icm20689_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;
	/*
	 * Just get the whoami
	 */
	rc = inv_icm20689_get_whoami(&icm_device, &whoami);
	check_rc(rc, "Error reading WHOAMI");
	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}
	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		check_rc(-1, "");
	}
	rc = inv_icm20689_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");

	/*
	 * Configure and initialize the icm20689 for normal use
	 */

	/* set default power mode */

	if(inv_icm20689_all_sensors_off(&icm_device)) {			
		inv_icm20689_init_matrix(&icm_device);
		icm20689_apply_mounting_matrix();
		rc = inv_icm20689_initialize(&icm_device, dmp3_image);
		check_rc(rc, "Error %d while setting-up device");
	}


	/* set default ODR = 50Hz */
	rc = inv_icm20689_set_sensor_period(&icm_device, INV_icm20689_SENSOR_RAW_ACCELEROMETER, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up device");

	rc = inv_icm20689_set_sensor_period(&icm_device, INV_icm20689_SENSOR_RAW_GYROSCOPE, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up device");
	//add run self test
	

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	return 0;
}

int icm20689_sensor_configuration(void)
{
	int rc;

	rc = inv_icm20689_set_accel_fullscale(&icm_device, inv_icm20689_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");

	rc = inv_icm20689_set_gyro_fullscale(&icm_device, inv_icm20689_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");

	return rc;
}

/*
 * Helper function to check RC value and block program execution
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		while(1);
	}
}



int sensor_configure_odr(uint32_t odr_us)
{
	int rc = 0;

	/* All sensors running at the same rate */

	/* Do not reconfigure the rate if it's already applied */
	if(odr_us == period_us)
		return rc;

	if(odr_us < MIN_ODR_US)
		odr_us = MIN_ODR_US;

	if(odr_us > MAX_ODR_US)
		odr_us = MAX_ODR_US;

	/* FIFO has been reset by ODR change */
	if (rc == 0) {

	}
	
	/* Keep track in static variable of the odr value for further algorihtm use */
	period_us = odr_us;

	return rc;
}

int sensor_control(int enable)
{
	int rc = 0;
	static uint8_t sensors_on = 0;

	/* Keep track of the sensors state */
	if(enable && sensors_on)
		return rc;

	if(enable)
		sensors_on = 1;
	else 
		sensors_on = 0;

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	if (enable) {
		/* Clock is more accurate when gyro is enabled, so let's enable it first to prevent side effect at startup */
		if (!inv_icm20689_is_sensor_enabled(&icm_device, INV_icm20689_SENSOR_RAW_GYROSCOPE))
			rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_RAW_GYROSCOPE, 1);

		if (!inv_icm20689_is_sensor_enabled(&icm_device, INV_icm20689_SENSOR_GYROSCOPE))
			rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_GYROSCOPE, 1);
		if (!inv_icm20689_is_sensor_enabled(&icm_device, INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED))
			rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED, 1);
			
		if (!inv_icm20689_is_sensor_enabled(&icm_device, INV_icm20689_SENSOR_RAW_ACCELEROMETER))
			rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_RAW_ACCELEROMETER, 1);
        //		if (!inv_icm20689_is_sensor_enabled(&icm_device, INV_icm20689_SENSOR_ACCELEROMETER))
        //			rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_ACCELEROMETER, 1);

		if (!inv_icm20689_is_sensor_enabled(&icm_device, INV_icm20689_SENSOR_GAME_ROTATION_VECTOR))
			rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_GAME_ROTATION_VECTOR, 1);
			
		/*
		 * There is a situation where two samples need to be dropped: if
		 * accelerometer is enable before gyroscope first interrupt triggers,
		 * both interrupts are raised causing the odr to be wrong if only one
		 * sample is dropped.
		 * We are in this exact situation since both sensors are enabled one after
		 * the other.
		 */
		//timestamp_to_drop = 2; //todo: is this needed? -- Qing
	} else {
		rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_RAW_ACCELEROMETER, 0);
//		rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_ACCELEROMETER, 0);
		
		rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_RAW_GYROSCOPE, 0);		
		rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_GYROSCOPE, 0);
		rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED, 0);

		rc += inv_icm20689_enable_sensor(&icm_device, INV_icm20689_SENSOR_GAME_ROTATION_VECTOR, 0);		
	}

	/* Clear the remaining items in the IRQ timestamp buffer when stopping all sensors */
	inv_icm20689_all_sensors_off(&icm_device);

	

	return rc;
}
static float angle[3];

/*
* Poll devices for data
*/
float* icm20689_data_poll(void) {

	int16_t int_read_back = 0;

	/*
	 *  Ensure data ready status
	 */
	inv_icm20689_identify_interrupt(&icm_device, &int_read_back);

	if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_2 | BIT_MSG_DMP_INT_3)) {
		inv_icm20689_poll_sensor(&icm_device, (void *)0, handle_data_function);
		
		return angle;
	}else
	{
		return NULL;
	}
}

static void icm20689_apply_mounting_matrix(void)
{
	int ii;
	
	for (ii = 0; ii < INV_icm20689_SENSOR_MAX; ii++) {
		inv_icm20689_set_matrix(&icm_device, cfg_mounting_matrix, ii);
	}
}
//inv_sensor_event_t event;

float pitch_save,yaw_save,roll_save;
void handle_data_function(void * context, uint8_t sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	float raw_bias_data[6];
	float quad[4]={0};
	uint8_t sensor_id = convert_to_generic_ids[sensortype];
	
	
	if(sensor_id == INV_SENSOR_TYPE_GAME_ROTATION_VECTOR)
	{
		memcpy(quad, data, sizeof(quad));
		angle[1] = asin(-2 * quad[1] * quad[3] + 2 * quad[0]* quad[2])* 57.3;	// pitch
		angle[2]  = atan2(2 * quad[2] * quad[3] + 2 * quad[0] * quad[1], -2 * quad[1] * quad[1] - 2 * quad[2]* quad[2] + 1)* 57.3;	// roll
		angle[0]  = atan2(2*(quad[1]*quad[2] + quad[0]*quad[3]),quad[0]*quad[0]+quad[1]*quad[1]-quad[2]*quad[2]-quad[3]*quad[3]) * 57.3;	//yaw
	}
}

void dmp_setup(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len))
{
	struct inv_icm20689_serif icm20689_serif;
	
	
	icm20689_serif.context   = 0; /* no need */
	icm20689_serif.read_reg  = read_reg;
	icm20689_serif.write_reg = write_reg;
	icm20689_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
	icm20689_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */
	icm20689_serif.is_spi    = 1;
	
	/* 
	 * Reset icm20689 driver states
	 */
	inv_icm20689_reset_states(&icm_device, &icm20689_serif);
	
	/*
	 * Setup the icm20689 device
	 */
	icm20689_sensor_setup();
	icm20689_sensor_configuration();
	icm20689_run_selftest();
	sensor_control(1);
	
	
}


int icm20689_run_selftest(void)
{
	int raw_bias[12];
	int rc = 0;

	if (icm_device.selftest_done == 1) {
	//	INV_MSG(INV_MSG_LEVEL_INFO, "Self-test has already run. Skipping.");
	}
	else {
		/* 
		 * Perform self-test
		 * For ICM20789 self-test is performed for both RAW_ACC/RAW_GYR
		 */
		//INV_MSG(INV_MSG_LEVEL_INFO, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm20689_run_selftest(&icm_device);
		/* Check transport errors */
		check_rc(rc, "Self-test failure");
		if (rc != 0x3) {
			/*
			 * Check for GYR success (1 << 0) and ACC success (1 << 1),
			 * but don't block as these are 'usage' failures.
			 */
			//INV_MSG(INV_MSG_LEVEL_ERROR, "Self-test failure");
			/* 0 would be considered OK, we want KO */
			return INV_ERROR;
		} else
			/* On success, offset will be kept until reset */
			icm_device.selftest_done = 1;

		/* It's advised to re-init the icm20789 device after self-test for normal use */
		rc = icm20689_sensor_setup();
	}

	/* 
	 * Get Low Noise / Low Power bias computed by self-tests scaled by 2^16
	 */
	//INV_MSG(INV_MSG_LEVEL_INFO, "Getting LP/LN bias");
	inv_icm20689_get_st_bias(&icm_device, raw_bias);
	//INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
	//		(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	//INV_MSG(INV_MSG_LEVEL_INFO, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
	//		(float)(raw_bias[3] / (float)(1 << 16)), (float)(raw_bias[4] / (float)(1 << 16)), (float)(raw_bias[5] / (float)(1 << 16)));
	//INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f", 
	//		(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
	//INV_MSG(INV_MSG_LEVEL_INFO, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f", 
	//		(float)(raw_bias[3 + 6] / (float)(1 << 16)), (float)(raw_bias[4 + 6] / (float)(1 << 16)), (float)(raw_bias[5 + 6] / (float)(1 << 16)));

	return rc;
}
