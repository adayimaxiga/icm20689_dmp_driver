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

#include "icm20689.h"
#include "icm20689Dmp3Driver.h"
#include "icm20689LoadFirmware.h"
#include "icm20689Defs.h"
#include "DataConverter.h"

/* invn.images.dmp.dmp3.20789_20689-0.4.1 */

#define CFG_OUT_STEPDET             (2237)
#define CFG_FB_GAIN_GYRO_ON         (1964)
#define CFG_OUT_GYRO                (2410)
#define GYRO_FIFO_RATE              (2396)
#define FSYNC_END                   (1347)
#define PREV_PQUAT                  (2437)
#define CFG_PEDSTEP_DET             (3376)
#define ACCEL_FIFO_RATE             (2361)
#define PED_STEP_COUNT2_DETECTED    (3382)
#define PQUAT_FIFO_RATE             (2326)
#define CFG_FIFO_INT                (2538)
#define CFG_AUTH                    (1366)
#define OUT_ACCL_DAT                (2385)
#define FCFG_3                      (1463)
#define FCFG_2                      (1437)
#define FCFG_1                      (1433)
#define FCFG_7                      (1444)
#define CFG_OUT_FSYNC               (1332)
#define CFG_OUT_ACCL                (2375)
#define CFG_OUT_3QUAT               (2268)
#define OUT_3QUAT_DAT               (2279)
#define CFG_PED_ENABLE              (2548)
#define CFG_EXT_GYRO_BIAS           (1552)
#define CFG_7                       (1900)
#define OUT_PQUAT_DAT               (2350)
#define CFG_OUT_6QUAT               (2304)
#define CFG_PED_INT                 (3365)
#define SMD_TP2                     (1840)
#define SMD_TP1                     (1817)
#define STEPDET_END                 (2254)
#define CFG_MOTION_BIAS             (1901)
#define OUT_6QUAT_DAT               (2315)
#define QUAT6_FIFO_RATE             (2290)
#define OUT_GYRO_DAT                (2420)
#define CFG_OUT_PQUAT               (2340)


#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       ((61 * 16) + 4)
#define D_EXT_GYRO_BIAS_Z       ((61 * 16) + 8)
#define GYRO_BIAS_X	D_EXT_GYRO_BIAS_X
#define GYRO_BIAS_Y	D_EXT_GYRO_BIAS_Y
#define GYRO_BIAS_Z	D_EXT_GYRO_BIAS_Z

#define D_EIS_ENABLE			(2 * 16 + 10)

#define D_SMD_ENABLE            (18 * 16)
#define D_SMD_MOT_THLD          (21 * 16 + 8)
#define SMD_VAR_TH	D_SMD_MOT_THLD

#define D_SMD_DELAY_THLD        (21 * 16 + 4)
#define D_SMD_DELAY2_THLD       (21 * 16 + 12)
#define D_SMD_EXE_STATE         (22 * 16)
#define D_SMD_DELAY_CNTR        (21 * 16)

#define D_VIB_DET_RATIO         (30 * 16 + 0)
#define D_VIB_MAG_TH            (31 * 16 + 0)
#define D_VIB_K                 (13 * 16 + 8)
#define D_VIB_K_1               (13 * 16 + 12)

#define D_ACC_DATA_SCALE		(15 * 16 + 4)
#define D_ACC_SCALE             (29 * 16 + 0)
#define ACC_SCALE D_ACC_SCALE

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)
#define D_DMP_AUTH_INPUT		(70 * 16 + 4)
#define D_DMP_AUTH_OUTPUT		(70 * 16 + 0)

#define SC_AUT_OUTPUT_20608D		D_DMP_AUTH_INPUT
#define SC_AUT_INPUT_20608D		D_DMP_AUTH_OUTPUT

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define PEDSTD_STEPCTR		D_PEDSTD_STEPCTR
#define D_PEDSTD_STEPCTR2       (58 * 16 + 8)
#define D_PEDSTD_TIMECTR        (964)
#define PEDSTD_TIMECTR		D_PEDSTD_TIMECTR
#define D_PEDSTD_DECI           (768 + 0xA0)
#define D_PEDSTD_SB2			(60 * 16 + 14)
#define D_STPDET_TIMESTAMP      (28 * 16 + 8)
#define STPDET_TIMESTAMP	D_STPDET_TIMESTAMP
#define D_PEDSTD_DRIVE_STATE    (58)
#define D_PEDSTEP_IND			(26*16 + 6)
#define PEDSTEP_IND				D_PEDSTEP_IND

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

// SW_KEY
#define D_SW_KEY                (28 * 16 + 14)

// Batch mode
#define D_BM_BATCH_CNTR         (27*16+4)
#define D_BM_BATCH_THLD         (27*16+12)
#define BM_BATCH_CNTR		D_BM_BATCH_CNTR
#define BM_BATCH_THLD		D_BM_BATCH_THLD

#define D_BM_ENABLE             (28*16+6)
#define D_BM_NUMWORD_TOFILL     (28*16+4)

// DMP running counter
#define D_DMP_RUN_CNTR          (24*16)
#define DMPRATE_CNTR		D_DMP_RUN_CNTR

// Sensor's ODR
#define D_ODR_S0                (45*16+8)
#define D_ODR_S1                (45*16+12)
#define D_ODR_S2                (45*16+10)
#define D_ODR_S3                (45*16+14)
#define D_ODR_S4                (46*16+8)
#define D_ODR_S5                (46*16+12)

#define D_ODR_S6                (46*16+10)
#define D_ODR_S7                (46*16+14)
#define D_ODR_S8                (42*16+8) 
#define D_ODR_S9                (42*16+12)

#define ODR_ACCEL			D_ODR_S4
#define ODR_GYRO			D_ODR_S5
#define ODR_QUAT6			D_ODR_S2
#define ODR_PQUAT6			D_ODR_S3

// sensor output data rate counter
#define D_ODR_CNTR_S0           (45*16)
#define D_ODR_CNTR_S1           (45*16+4)
#define D_ODR_CNTR_S2           (45*16+2)
#define D_ODR_CNTR_S3           (45*16+6)
#define D_ODR_CNTR_S4           (46*16)
#define D_ODR_CNTR_S5           (46*16+4)
#define D_ODR_CNTR_S6           (46*16+2)
#define D_ODR_CNTR_S7           (46*16+6)
#define D_ODR_CNTR_S8           (42*16)
#define D_ODR_CNTR_S9           (42*16+4)

#define ODR_CNTR_ACCEL				D_ODR_CNTR_S4
#define ODR_CNTR_GYRO				D_ODR_CNTR_S5
#define ODR_CNTR_QUAT6				D_ODR_CNTR_S2
#define ODR_CNTR_PQUAT6			D_ODR_CNTR_S3

// DMP Fusion LP-Quat
#define D_FS_LPQ0               (59*16)
#define D_FS_LPQ1               (59*16 + 4)
#define D_FS_LPQ2               (59*16 + 8)
#define D_FS_LPQ3               (59*16 + 12)

// DMP Fusion (Accel+Gyro) Quat
#define D_ACCEL_FB_GAIN			(6*16)
#define D_GYRO_SF				(6*16 + 8)
#define GYRO_SF		D_GYRO_SF
#define D_FS_Q0                 (12*16)
#define D_FS_Q1                 (12*16 + 4)
#define D_FS_Q2                 (12*16 + 8)
#define D_FS_Q3                 (12*16 + 12)

// AK compass support
#define D_CPASS_STATUS_CHK		(22*16 + 8)

#define ACCEL_SET			0x4000
#define GYRO_SET			0x2000
#define QUAT6_SET			0x0400
#define PQUAT6_SET			0x0200
#define PED_STEPDET_SET		0x0100
#define HEADER2_SET			0x0008
#define PED_STEPIND_SET		0x0007
#define FSYNC_HDR           0x7000

// high byte of motion event control
#define PEDOMETER_EN        0x4000
#define PEDOMETER_INT_EN    0x2000
#define TILT_INT_EN         0x1000
#define SMD_EN              0x0800
#define SECOND_SENSOR_AUTO  0x0400
#define ACCEL_CAL_EN        0x0200
#define GYRO_CAL_EN         0x0100
// low byte of motion event control
#define COMPASS_CAL_EN      0x0080
#define NINE_AXIS_EN        0x0040
#define S_HEALTH_EN         0x0020
#define FLIP_PICKUP_EN      0x0010
#define GEOMAG_RV_EN        0x0008
#define BRING_LOOK_SEE_EN   0x0004
#define BAC_ACCEL_ONLY_EN   0x0002

#define DMP_OFFSET               0x20
//#define DMP_IMAGE_SIZE_20789     (3543 + DMP_OFFSET)
#define DMP_START_ADDR_20789     0x4B0
//#define DMP_CODE_SIZE  DMP_IMAGE_SIZE_20789-DMP_OFFSET
#define DMP_CODE_SIZE 3548

/** Loads the dmp firmware for the icm20689 part.
* @param[in] dmp_image_sram Load DMP3 image from SRAM.
*/
int inv_icm20689_load_firmware(struct inv_icm20689 * s, const unsigned char *dmp3_image)
{
	return inv_icm20689_firmware_load(s, dmp3_image, DMP_CODE_SIZE, DMP_OFFSET);
}

/** Loads the dmp firmware for the icm20689 part.
* @param[out] dmp_cnfg The config item
*/
void inv_icm20689_get_dmp_start_address(struct inv_icm20689 * s, unsigned short *dmp_cnfg)
{
	(void)s;

	*dmp_cnfg = DMP_START_ADDR_20789;
}

void dmp_icm20689_get_sw_key_addr(struct inv_icm20689 * s, unsigned short *key_addr)
{
	(void)s;

	*key_addr = D_SW_KEY;
}

/* set SW_KEY (WHO_AM_I) */
int dmp_icm20689_set_sw_key(struct inv_icm20689 * s, unsigned char key)
{
	unsigned char reg[2]= {0};
	int result;

	reg[1] = key;

	result = inv_icm20689_write_mems(s, D_SW_KEY, 2, &reg[0]);

	return result;	
}

int inv_enable_gyro_cal(struct inv_icm20689 * s, int en)
{
	unsigned char reg[3] = {0xc2, 0xc5, 0xc7};
	int result;

	if (!en) {
		reg[0] = 0xf1;
		reg[1] = 0xf1;
		reg[2] = 0xf1;
	}

	result = inv_icm20689_write_mems(s, CFG_EXT_GYRO_BIAS, 3, &reg[0]);

	return result;
}

int inv_send_accel_data(struct inv_icm20689 * s, int enable)
{
	unsigned char reg[3] = {0xa3, 0xa3, 0xa3};
	int result;

	/* turning off accel jumps to GYRO_FIFO_RATE */
	if (!enable) {
		reg[0] = 0xf4;
		reg[1] = (GYRO_FIFO_RATE >> 8) & 0xff;
		reg[2] = GYRO_FIFO_RATE & 0xff;
	}
	result = inv_icm20689_write_mems(s, CFG_OUT_ACCL, 3, reg);

	return result;
}

int inv_send_gyro_data(struct inv_icm20689 * s, int enable)
{
	unsigned char reg[3] = {0xa3, 0xa3, 0xa3};
	int result;

	/* turning off gyro jumps to PREV_PQUAT */
	if (!enable) {
		reg[0] = 0xf4;
		reg[1] = (PREV_PQUAT >> 8) & 0xff;
		reg[2] = PREV_PQUAT & 0xff;
	}
	result = inv_icm20689_write_mems(s, CFG_OUT_GYRO, 3, reg);

	return result;
}

int inv_send_six_q_data(struct inv_icm20689 * s, int enable)
{
	unsigned char reg[3] = {0xa3, 0xa3, 0xa3};
	int result;

	/* turning off 6-axis jumps to PQUAT_FIFO_RATE */
	if (!enable) {
		reg[0] = 0xf4;
		reg[1] = (PQUAT_FIFO_RATE >> 8) & 0xff;
		reg[2] = PQUAT_FIFO_RATE & 0xff;
	}
	result = inv_icm20689_write_mems(s, CFG_OUT_6QUAT, 3, reg);

	return result;
}

/**
* Sets data output control register 1.
* @param[in] output_mask	Turns sensors on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*							DMP will also turn hw sensors on/off based on bits set in output_mask.
*
*	ACCEL_SET				0x4000 - raw accel
*	GYRO_SET				0x2000 - gyro + bias
*	QUAT6_SET				0x0400 - game rotation vector
*	PQUAT6_SET				0x0200
*	PED_STEPDET_SET			0x0100
*	HEADER2_SET				0x0008
*	PED_STEPIND_SET			0x0007
*	FSYNC_HDR				0x7000
*/
int dmp_icm20689_set_data_output_control1(struct inv_icm20689 * s, int output_mask)
{  
	(void)s;
	int result = 0;

	if (output_mask & ACCEL_SET)
	result = inv_send_accel_data(s, 1);
	else
	result = inv_send_accel_data(s, 0);

	if (output_mask & GYRO_SET)
	result |= inv_send_gyro_data(s, 1);
	else
	result |= inv_send_gyro_data(s, 0);

	if (output_mask & QUAT6_SET)
	result |= inv_send_six_q_data(s, 1);
	else
	result |= inv_send_six_q_data(s, 0);
/*
	if (output_mask & PQUAT6_SET)
	result |= inv_send_ped_q_data(1);
	else
	result |= inv_send_ped_q_data(0);

	if ((output_mask & PED_STEPDET_SET)) {
		result |= inv_enable_pedometer(1);
		result |= inv_enable_pedometer_interrupt(1);
		result |= inv_send_stepdet_data(1);
		} else {
		result |= inv_enable_pedometer(0);
		result |= inv_enable_pedometer_interrupt(0);
		result |= inv_send_stepdet_data(0);
	}
*/
	return result;
}

/**
* Sets data output control register 2.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*	ACCEL_ACCURACY_SET	0x4000 - accel accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	GYRO_ACCURACY_SET	0x2000 - gyro accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	CPASS_ACCURACY_SET	0x1000 - compass accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	BATCH_MODE_EN		0x0100 - enable batching
*/
int dmp_icm20689_set_data_output_control2(struct inv_icm20689 * s, int output_mask)
{
	int result = 0;
/*
	int en;
	
	if (output_mask & BATCH_MODE_EN)
		en = 1;
	else
		en = 0;
	result = inv_enable_batch(en);
	if (result)
		return result;

	if (output_mask & FSYNC_SET)
		en = 1;
	else
		en = 0;
	result = inv_enable_eis(en);
	if (result)
		return result;
	result = inv_out_fsync(en);
	if (result)
		return result;
*/
	return result;
}

int dmp_icm20689_reset_odr_counters(struct inv_icm20689 * s)
{
	int result;
	unsigned char data[4] = {0x00,0x00,0x00,0x00};
	result = inv_icm20689_write_mems(s, ODR_CNTR_ACCEL, 4, data);
	result |= inv_icm20689_write_mems(s, ODR_CNTR_GYRO, 4, data);
	result |= inv_icm20689_write_mems(s, ODR_CNTR_QUAT6, 4, data);
	result |= inv_icm20689_write_mems(s, ODR_CNTR_PQUAT6, 4, data);
	return result;
}

/**
* Clears all output control registers:
*	data output control register 1, data output control register 2, data interrupt control register, motion event control regsiter, data ready status register
*/
int dmp_icm20689_reset_control_registers(struct inv_icm20689 * s)
{
	return 0;
}

/**
* Sets data interrupt control register.
* @param[in] interrupt_ctl	Determines which sensors can generate interrupt according to following bit definition,
*							bit set indicates interrupt, bit clear indicates no interrupt.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - data output defined in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
int dmp_icm20689_set_data_interrupt_control(struct inv_icm20689 * s, uint32_t interrupt_ctl)
{
	return 0;
}

/**
* Sets FIFO watermark. DMP will send FIFO interrupt if FIFO count > FIFO watermark
* @param[in] fifo_wm	FIFO watermark set to 80% of actual FIFO size by default
*/
int dmp_icm20689_set_FIFO_watermark(struct inv_icm20689 * s, unsigned short fifo_wm)
{
	return 0;
}

/**
* Sets data rdy status register.
* @param[in] data_rdy	Indicates which sensor data is available.
*
*	gyro samples available		0x1
*	accel samples available		0x2
*	secondary samples available	0x8
*/
int dmp_icm20689_set_data_rdy_status(struct inv_icm20689 * s, unsigned short data_rdy)
{
	return 0;
}

/**
* Sets motion event control register.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*   BAC_WEAR_EN         0x8000 - change BAC behavior for wearable platform
*	PEDOMETER_EN		0x4000 - pedometer engine
*	PEDOMETER_INT_EN	0x2000 - pedometer step detector interrupt
*	SMD_EN				0x0800 - significant motion detection interrupt
*	ACCEL_CAL_EN		0x0200 - accel calibration
*	GYRO_CAL_EN			0x0100 - gyro calibration
*	COMPASS_CAL_EN		0x0080 - compass calibration
*	NINE_AXIS_EN        0x0040 - 9-axis algorithm execution
*	GEOMAG_EN			0x0008 - Geomag algorithm execution
*	BTS_LTS_EN          0x0004 - bring & look to see
*	BAC_ACCEL_ONLY_EN   0x0002 - run BAC as accel only
*/
int dmp_icm20689_set_motion_event_control(struct inv_icm20689 * s, unsigned short output_mask)
{
	int result = 0;
/*
	if (output_mask & PEDOMETER_EN)
		result = inv_enable_pedometer(1);
	else
		result = inv_enable_pedometer(0);
*/
	/*if (output_mask & PEDOMETER_INT_EN)
		result = inv_enable_pedometer_interrupt(1);
	else
		result = inv_enable_pedometer_interrupt(0);*/
/*
	if (output_mask & SMD_EN)
		result = inv_enable_smd(1);
	else
		result = inv_enable_smd(0);
*/
	if (output_mask & GYRO_CAL_EN)
		result = inv_enable_gyro_cal(s, 1);
	else
		result = inv_enable_gyro_cal(s, 0);

	return result;
}

/**
* Sets sensor ODR.
* @param[in] sensor		sensor number based on INV_SENSORS
*	enum INV_SENSORS {
*		INV_SENSOR_ACCEL = 0,
*		INV_SENSOR_GYRO,        
*	    INV_SENSOR_LPQ,
*		INV_SENSOR_COMPASS,
*		INV_SENSOR_ALS,
*		INV_SENSOR_SIXQ,
*		INV_SENSOR_NINEQ,
*		INV_SENSOR_GEOMAG,
*		INV_SENSOR_PEDQ,
*		INV_SENSOR_PRESSURE,
*		INV_SENSOR_CALIB_GYRO,
*		INV_SENSOR_CALIB_COMPASS,
*		INV_SENSOR_NUM_MAX,
*		INV_SENSOR_INVALID,
*	};					
* @param[in] divider	desired ODR = base engine rate/(divider + 1)
*/
int dmp_icm20689_set_sensor_rate(struct inv_icm20689 * s, int invSensor, short divider)
{
	int result = 0;
	unsigned char data[2] = {0x00,0x00};
	unsigned char big8[2]={0};
	int odr_cntr_addr;
	int odr_addr;

	switch (invSensor) {
		case INV_SENSOR_ACCEL:
			odr_cntr_addr = ODR_CNTR_ACCEL;
			odr_addr = ODR_ACCEL;
			break;
		case INV_SENSOR_GYRO:
			odr_cntr_addr = ODR_CNTR_GYRO;
			odr_addr = ODR_GYRO;
			break;
		case INV_SENSOR_SIXQ:
			odr_cntr_addr = ODR_CNTR_QUAT6;
			odr_addr = ODR_QUAT6;
			break;
		case INV_SENSOR_PEDQ:
			odr_cntr_addr = ODR_CNTR_PQUAT6;
			odr_addr = ODR_PQUAT6;
			break;
	}

	result |= inv_icm20689_write_mems(s, odr_cntr_addr, 2, data);
	result |= inv_icm20689_write_mems(s, odr_addr, 2, inv_dc_int16_to_big8(divider, big8));

	return 0;
}

/**
* Sets acc's bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*/
int dmp_icm20689_set_bias_acc(struct inv_icm20689 * s, int *bias)
{
	// todo
	return 0; 
}

/**
* Sets gyro's bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] gyro_x
*	[1] gyro_y
*	[2] gyro_z
*/
int dmp_icm20689_set_bias_gyr(struct inv_icm20689 * s, int *bias)
{
	// todo
	return 0; 
}


/**
* Gets acc's bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*/
int dmp_icm20689_get_bias_acc(struct inv_icm20689 * s, int *bias)
{
	// todo
	return 0; 
}

/**
* Gets gyro's bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] gyro_x
*	[1] gyro_y
*	[2] gyro_z
*/
int dmp_icm20689_get_bias_gyr(struct inv_icm20689 * s, int *bias)
{
	// todo
	return 0; 
}

/**
* Sets the gyro_sf used by quaternions on the DMP.
* @param[in] gyro_sf	see inv_icm20689_set_gyro_sf() for value to set based on gyro rate and gyro fullscale range
*/
int dmp_icm20689_set_gyro_sf(struct inv_icm20689 * s, long gyro_sf)
{
    return 0;
}

/**
* Sets the accel gain used by accel quaternion on the DMP.
* @param[in] accel_gain		value changes with accel engine rate
*/
int dmp_icm20689_set_accel_feedback_gain(struct inv_icm20689 * s, int accel_gain)
{
	return 0;
}

/**
* Sets accel cal parameters based on different accel engine rate/accel cal running rate
* @param[in] accel_cal
*	array is set as follows:
*	[0] = ACCEL_CAL_ALPHA_VAR
*	[1] = ACCEL_CAL_A_VAR
*   [2] = ACCEL_CAL_DIV - divider from hardware accel engine rate such that acce cal runs at accel_engine_rate/(divider+1)
*/
int dmp_icm20689_set_accel_cal_params(struct inv_icm20689 * s, int *accel_cal)
{
	return 0;
}

/**
* Gets pedometer step count.
* @param[in] steps
* @param[out] steps
*/
int dmp_icm20689_get_pedometer_num_of_steps(struct inv_icm20689 * s, unsigned long *steps)
{
	// todo
    
    return 0;
}

/**
* Sets pedometer engine running rate.
* @param[in] ped_rate	divider based on accel engine rate
*/
int dmp_icm20689_set_pedometer_rate(struct inv_icm20689 * s, int ped_rate)
{
	// int result; 
	// unsigned char big8[4]={0};
	// result = inv_icm20689_write_mems(s, PED_RATE, 4, inv_icm20689_convert_int32_to_big8(ped_rate, big8));
	// if (result)
	//    return result;

	(void)s;
	(void) ped_rate;

	return 0;
}

/**
* Sets scale in DMP to convert accel data to 1g=2^25 regardless of fsr.
* @param[in] fsr for accel parts
             2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.

             For 2g parts, 2g = 2^15 -> 1g = 2^14,.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^14 (<<16) becomes 1g=2^30, to make 1g=2^25, >>5 bits.
             In Q-30 math, >> 5 equals multiply by 2^25 = 33554432.

             For 8g parts, 8g = 2^15 -> 1g = 2^12.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^12 (<<16) becomes 1g=2^28, to make 1g=2^25, >>3bits.
             In Q-30 math, >> 3 equals multiply by 2^27 = 134217728.
*/
int dmp_icm20689_set_accel_fsr(struct inv_icm20689 * s, short accel_fsr)
{
    return 0;
}

/**
* According to input fsr, a scale factor will be set at memory location ACC_SCALE2
* to convert calibrated accel data to 16-bit format same as what comes out of MPU register.
* It is a reverse scaling of the scale factor written to ACC_SCALE.
* @param[in] fsr for accel parts
			 2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.
*/
int dmp_icm20689_set_accel_scale2(struct inv_icm20689 * s, short accel_fsr)
{
    return 0;
}

