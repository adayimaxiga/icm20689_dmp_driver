/*
* ________________________________________________________________________________________________________
* Copyright © 2014 InvenSense Inc.  All rights reserved.
*
* This software and/or documentation  (collectively “Software”) is subject to InvenSense intellectual property rights 
* under U.S. and international copyright and other intellectual property rights laws.
*
* The Software contained herein is PROPRIETARY and CONFIDENTIAL to InvenSense and is provided 
* solely under the terms and conditions of a form of InvenSense software license agreement between 
* InvenSense and you and any use, modification, reproduction or disclosure of the Software without 
* such agreement or the express written consent of InvenSense is strictly prohibited.
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

#ifndef _INV_icm20689_DMP3_DRIVER_H_
#define _INV_icm20689_DMP3_DRIVER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* invn.images.dmp.dmp3.20789_20689-0.4.1 */

/* forward declaration */
struct inv_icm20689;

/* enum for sensor
   The sequence is important.
   It represents the order of appearance from DMP */
enum INV_SENSORS {
	INV_SENSOR_ACCEL = 0,
	INV_SENSOR_GYRO,        
	INV_SENSOR_LPQ,             // 20610:  we'll find out if it breaks 20628 being inserted here....       
	INV_SENSOR_COMPASS,
	INV_SENSOR_ALS,
	INV_SENSOR_SIXQ,
	INV_SENSOR_NINEQ,
	INV_SENSOR_GEOMAG,
INV_SENSOR_PEDQ,
INV_SENSOR_PRESSURE,
	INV_SENSOR_CALIB_GYRO,
	INV_SENSOR_CALIB_COMPASS,
	INV_SENSOR_STEP_COUNTER,
	INV_SENSOR_ACTIVITY_CLASSIFIER,
	INV_SENSOR_FLIP_PICKUP,
    INV_SENSOR_BRING_TO_SEE,

INV_SENSOR_SIXQ_accel,
INV_SENSOR_NINEQ_accel,
INV_SENSOR_GEOMAG_cpass,
INV_SENSOR_NINEQ_cpass,

	INV_SENSOR_WAKEUP_ACCEL,
	INV_SENSOR_WAKEUP_GYRO,        
//INV_SENSOR_WAKEUP_LPQ,
	INV_SENSOR_WAKEUP_COMPASS,
	INV_SENSOR_WAKEUP_ALS,
	INV_SENSOR_WAKEUP_SIXQ,
	INV_SENSOR_WAKEUP_NINEQ,
	INV_SENSOR_WAKEUP_GEOMAG,
INV_SENSOR_WAKEUP_PEDQ,
INV_SENSOR_WAKEUP_PRESSURE,
	INV_SENSOR_WAKEUP_CALIB_GYRO,
	INV_SENSOR_WAKEUP_CALIB_COMPASS,
	INV_SENSOR_WAKEUP_STEP_COUNTER,
	INV_SENSOR_WAKEUP_TILT_DETECTOR,
//INV_SENSOR_WAKEUP_ACTIVITY_CLASSIFIER,

INV_SENSOR_WAKEUP_SIXQ_accel,
INV_SENSOR_WAKEUP_NINEQ_accel,
INV_SENSOR_WAKEUP_GEOMAG_cpass,
INV_SENSOR_WAKEUP_NINEQ_cpass,

	INV_SENSOR_NUM_MAX,
	INV_SENSOR_INVALID,
};


enum accel_cal_params {
    ACCEL_CAL_ALPHA_VAR = 0,
    ACCEL_CAL_A_VAR,
    ACCEL_CAL_DIV,
    NUM_ACCEL_CAL_PARAMS
};

enum compass_cal_params {
	CPASS_CAL_TIME_BUFFER = 0,
	CPASS_CAL_RADIUS_3D_THRESH_ANOMALY,
    NUM_CPASS_CAL_PARAMS
};

int inv_enable_gyro_cal(struct inv_icm20689 * s, int en);
int inv_send_accel_data(struct inv_icm20689 * s, int enable);
int inv_send_gyro_data(struct inv_icm20689 * s, int enable);
int inv_send_six_q_data(struct inv_icm20689 * s, int enable);

int inv_icm20689_load_firmware(struct inv_icm20689 * s, const unsigned char *dmp3_image);
void inv_icm20689_get_dmp_start_address(struct inv_icm20689 * s, unsigned short *dmp_cnfg);
void dmp_icm20689_get_sw_key_addr(struct inv_icm20689 * s, unsigned short *key_addr);
int dmp_icm20689_set_sw_key(struct inv_icm20689 * s, unsigned char key);
int dmp_icm20689_reset_odr_counters(struct inv_icm20689 * s);
int dmp_icm20689_reset_control_registers(struct inv_icm20689 * s);
int dmp_icm20689_set_data_output_control1(struct inv_icm20689 * s, int output_mask);
int dmp_icm20689_set_data_output_control2(struct inv_icm20689 * s, int output_mask);
int dmp_icm20689_set_data_interrupt_control(struct inv_icm20689 * s, uint32_t interrupt_ctl);
int dmp_icm20689_set_FIFO_watermark(struct inv_icm20689 * s, unsigned short fifo_wm);
int dmp_icm20689_set_data_rdy_status(struct inv_icm20689 * s, unsigned short data_rdy);
int dmp_icm20689_set_motion_event_control(struct inv_icm20689 * s, unsigned short motion_mask);
int dmp_icm20689_set_sensor_rate(struct inv_icm20689 * s, int sensor, short divider);
int dmp_icm20689_set_bias_acc(struct inv_icm20689 * s, int *bias);
int dmp_icm20689_set_bias_gyr(struct inv_icm20689 * s, int *bias);
int dmp_icm20689_get_bias_acc(struct inv_icm20689 * s, int *bias);
int dmp_icm20689_get_bias_gyr(struct inv_icm20689 * s, int *bias);
int dmp_icm20689_set_gyro_sf(struct inv_icm20689 * s, long gyro_sf);
int dmp_icm20689_set_accel_feedback_gain(struct inv_icm20689 * s, int accel_gain);
int dmp_icm20689_set_accel_cal_params(struct inv_icm20689 * s, int *accel_cal);
int dmp_icm20689_get_pedometer_num_of_steps(struct inv_icm20689 * s, unsigned long *steps);
int dmp_icm20689_set_pedometer_rate(struct inv_icm20689 * s, int ped_rate);
int dmp_icm20689_set_gyro_fsr(struct inv_icm20689 * s, short gyro_fsr);
int dmp_icm20689_set_accel_fsr(struct inv_icm20689 * s, short accel_fsr);
int dmp_icm20689_set_accel_scale2(struct inv_icm20689 * s, short accel_fsr);


#ifdef __cplusplus
}
#endif

#endif /* _INV_icm20689_DMP3_DRIVER_H_ */
