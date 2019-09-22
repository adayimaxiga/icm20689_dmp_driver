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

/** @defgroup Drivericm20689Ctrl icm20689 control
 *  @brief Low-level function to control a icm20689 device
 *  @ingroup  Drivericm20689
 *  @{
 */
#ifndef _INV_icm20689_CTRL_H_
#define _INV_icm20689_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "InvExport.h"

#include <stdint.h>

/* forward declaration */
struct inv_icm20689;

/** @brief Sensor identifier for control function
 */
enum inv_icm20689_sensor {
	/*INV_icm20689_SENSOR_ACCELEROMETER,*/
	INV_icm20689_SENSOR_GYROSCOPE,
	INV_icm20689_SENSOR_RAW_ACCELEROMETER,
	INV_icm20689_SENSOR_RAW_GYROSCOPE,
	INV_icm20689_SENSOR_GYROSCOPE_UNCALIBRATED,
	INV_icm20689_SENSOR_GAME_ROTATION_VECTOR,
	INV_icm20689_SENSOR_CUSTOM_PRESSURE,
	INV_icm20689_SENSOR_MAX,
};

// data output control reg 1
#define ACCEL_SET			0x4000
#define GYRO_SET			0x2000
#define QUAT6_SET			0x0400
#define PQUAT6_SET			0x0200
#define PED_STEPDET_SET		0x0100
#define HEADER2_SET			0x0008
#define FSYNC_HDR           0x7000

#define GYRO_CALIBR_SET	0x0040
#define PED_STEPIND_SET 0x0007

// data output control reg 2
#define ACCEL_ACCURACY_SET	0x4000
#define GYRO_ACCURACY_SET	0x2000
#define BATCH_MODE_EN		0x0100

// motion event control reg
#define INV_PEDOMETER_EN		0x4000
#define INV_PEDOMETER_INT_EN	0x2000
#define INV_SMD_EN				0x0800
#define INV_ACCEL_CAL_EN		0x0200
#define INV_GYRO_CAL_EN			0x0100

#define INV_BTS_EN				0x1000

// data packet size reg 1
#define HEADER_SZ		2
#define ACCEL_DATA_SZ	6
#define GYRO_DATA_SZ    6
#define QUAT6_DATA_SZ	14

#define PQUAT6_DATA_SZ	6
#define GYRO_CALIBR_DATA_SZ		12
#define GYRO_BIAS_DATA_SZ 12
#define PED_STEPDET_TIMESTAMP_SZ	(4 + HEADER_SZ)

// data packet size reg 2
#define HEADER2_SZ			2
#define ACCEL_ACCURACY_SZ	2
#define GYRO_ACCURACY_SZ	2
#define CPASS_ACCURACY_SZ	2
#define FSYNC_SZ			2
#define FLIP_PICKUP_SZ      2
#define ACT_RECOG_SZ        6

/** @brief Enables accel and/or gyro and/or pressure if integrated with gyro and accel.
 *  @param[in] bit_mask A mask where 2 means turn on accel, 1 means turn on gyro, 4 is for pressure.
 *            			By default, this only turns on a sensor if all sensors are off otherwise the DMP controls
 *                      this register including turning off a sensor. To override this behavior add in a mask of 128.
 *  @param[in] smplrt_divider  The divider which was applied to internal sample rate based on field sample_rate
 *                            from base_driver_t to get minimum ODR for accel and gyro
 *  @return 0 on success, negative value on error
 */
int INV_EXPORT inv_icm20689_enable_mems(struct inv_icm20689 * s, int bit_mask, uint16_t smplrt_divider);

/** @brief Sets the odr for a sensor
 *  @param[in] sensor  Sensor Identity
 *  @param[in] delayInMs  the delay between two values in ms
 *  @return 0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20689_set_sensor_period(struct inv_icm20689 * s, enum inv_icm20689_sensor sensor, uint16_t delayInMs);

/** @brief Knows if the sensor is enabled or disabled
 *  @param[in] sensor  the sensor is enabled or not
 *  @return 1 if active, 0 otherwise
 */
int INV_EXPORT inv_icm20689_is_sensor_enabled(struct inv_icm20689 * s, enum inv_icm20689_sensor sensor);

/** @brief Enables / disables a sensor
 * @param[in] androidSensor  Sensor Identity
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int INV_EXPORT inv_icm20689_enable_sensor(struct inv_icm20689 * s, enum inv_icm20689_sensor sensor, uint8_t enable);

/** @brief  Configures accel WOM.
 *  @param[in] wom_threshold threshold value for X,Y,Z axis that should trigger a WOM interrupt (0 to diable WOM)
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_configure_accel_wom(struct inv_icm20689 * s, uint8_t wom_threshold);

/** @brief Check and retrieve for new data
 */
struct inv_icm20689_fifo_states {
	uint8_t overflow:1; /**< indicates FIFO overflow: user should restart all sensors if this occurs */
	uint16_t sensor_on_mask;
	uint16_t packet_count_i;
	uint16_t packet_count;
	uint16_t packet_size;
};

/** @brief Check for WOM bits in INT register value
 *  @param[in] int_status INT status register value
 *  @return bit mask corresponding to x, y, z axis that caused WOM
 *          (0x01: x axis, 0x02: y axis, 0x01: z axis)
 */
int INV_EXPORT inv_icm20689_check_wom_status(struct inv_icm20689 * s, uint8_t int_status);

/** @brief Disable FIFO
 * @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_disable_fifo(struct inv_icm20689 * s);

int INV_EXPORT inv_icm20689_disable_dmp(struct inv_icm20689 * s);

/** @brief    Enable FIFO
 * @return    0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_enable_fifo(struct inv_icm20689 * s);

int INV_EXPORT inv_icm20689_enable_dmp(struct inv_icm20689 * s);

int INV_EXPORT inv_icm20689_reset_dmp(struct inv_icm20689 * s);

/** @brief  Reset FIFO
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_reset_fifo(struct inv_icm20689 * s);

/** @brief  test if all sensors are off
 *  @return true if all sensors are off, false otherwise
 */
int INV_EXPORT inv_icm20689_all_sensors_off(struct inv_icm20689 * s);

int INV_EXPORT inv_icm20689_poll_sensor(struct inv_icm20689 * s, void * context,
void (*handler)(void * context, enum inv_icm20689_sensor sensor, uint64_t timestamp, const void * data, const void *arg));

#ifdef __cplusplus
}
#endif

#endif /* _INV_icm20689_CTRL_H_ */

/** @} */
