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

/** @defgroup Drivericm20689Setup icm20689 driver setup
 *  @brief Low-level function to setup an icm20689 device
 *  @ingroup  Drivericm20689
 *  @{
 */

#ifndef _INV_icm20689_SETUP_H_
#define _INV_icm20689_SETUP_H_

#include "InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "InvBool.h"

/* forward declaration */
struct inv_icm20689;
enum inv_icm20689_sensor;

/** @brief return WHOAMI value
 *  @param[out] whoami WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_icm20689_get_whoami(struct inv_icm20689 * s, uint8_t * whoami);


/** @brief return WHOAMI value
 *  @param[out] chip_info[0] WHOAMI for device
 *  @param[out] chip_info[1] MANUFACTURER_ID for device
 *  @param[out] chip_info[2] CHIP_ID for device
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_icm20689_get_chip_info(struct inv_icm20689 * s, uint8_t chip_info[3]);

/** @brief Sets up dmp start address and firmware
 *  @return  0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_set_dmp_address(struct inv_icm20689 * s);

void INV_EXPORT inv_icm20689_init_matrix(struct inv_icm20689 * s);

int INV_EXPORT inv_icm20689_set_matrix(struct inv_icm20689 * s, float matrix[9], enum inv_icm20689_sensor sensor);

/** @brief Initialize the device
 *  @param[in] dmp_image_sram DMP3 image to be loaded.
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_initialize(struct inv_icm20689 * s, const unsigned char *dmp_image_sram);

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_soft_reset(struct inv_icm20689 * s);

/** @brief Set the mpu sample rate
 *  @return Value written to MPUREG_SMPLRT_DIV register.
 */
int INV_EXPORT inv_icm20689_set_divider(struct inv_icm20689 * s, uint8_t idiv);

/** @brief Sets the power state of the Ivory chip loop
 *  @param[in] func   CHIP_AWAKE, CHIP_LP_ENABLE
 *  @param[in] on_off The functions are enabled if previously disabled and
 *                	  disabled if previously enabled based on the value of On/Off.
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_set_chip_power_state(struct inv_icm20689 * s, uint8_t func, uint8_t on_off);

/** @brief Current wake status of the Mems chip
 * @return the wake status
 */
uint8_t INV_EXPORT inv_icm20689_get_chip_power_state(struct inv_icm20689 * s);

/** @brief Get internal sample rate currently configured
 * @return Internal sample rate currently configured in MEMS registers in Hz
 */
uint16_t INV_EXPORT inv_icm20689_get_chip_base_sample_rate(struct inv_icm20689 * s);

/** @brief Get internal register value for given FSR in mg for Accelerometer
 *  Allowed value are: 2000 (+/-2g), 4000 (+/-4g), 8000 (+/-8g), 16000 (+/-16g), 
 *  @return internal register value for FSR configuration
 */
int INV_EXPORT inv_icm20689_accel_fsr_2_reg(int32_t fsr);

/** @brief Get FSR value in mg corresponding to internal register value for Accelerometer
 *  Allowed value are: 0 (+/-2g), 1 (+/-4g), 2 (+/-8g), 3 (+/-16g), 
 *  @return FSR value in mg
 */
int INV_EXPORT inv_icm20689_reg_2_accel_fsr(uint8_t reg);

/** @brief Sets fullscale range of accel in hardware.
 * @param[in] level  See mpu_accel_fs.
 * @return 			0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_set_accel_fullscale(struct inv_icm20689 * s, int level);

/** @brief Sets bandwidth range of accel in hardware.
 * @param[in] level  See mpu_accel_bw.
 * @return 			0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_set_accel_bandwidth(struct inv_icm20689 * s, int level);

/** @brief Returns fullscale range of accelerometer in hardware
 *  @return the fullscale range
 */
uint8_t INV_EXPORT inv_icm20689_get_accel_fullscale(struct inv_icm20689 * s);

/** @brief Returns bandwidth range of accelerometer in hardware
 *  @return the bandwidth range
 */
uint16_t INV_EXPORT inv_icm20689_get_accel_bandwidth(struct inv_icm20689 * s);

/** @brief Get internal register value for given FSR in dps for Gyroscope
 *  Allowed value are: 250 (+/-250dps), 500 (+/-500dps), 1000 (+/-1000dps), 2000 (+/-2000dps), 
 *  31 and 32 (+/-31.25dps), 62 and 63 (+/-62.5dps), 125 (+/-125dps)
 *  @return internal register value for FSR configuration
 */
int INV_EXPORT inv_icm20689_gyro_fsr_2_reg(int32_t fsr);

/** @brief Get FSR value in dps corresponding to internal register value for Gyroscope
 *  Allowed value are: 0 (+/-250dps), 1 (+/-500dps), 2 (+/-1000dps), 3 (+/-2000dps), 
 *  5 (+/-31.25dps), 6 (+/-62.5dps), 7 (+/-125dps)
 *  @return FSR value in dps
 */
int INV_EXPORT inv_icm20689_reg_2_gyro_fsr(uint8_t reg);

/** @brief Sets fullscale range of gyro in hardware.
 *  @param[in]  level  See mpu_gyro_fs.
 *  @return 				0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_set_gyro_fullscale(struct inv_icm20689 * s, int level);

/** @brief Sets bandwidth range of gyro in hardware.
 *  @param[in]  level  See mpu_gyro_bw.
 *  @return 				0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20689_set_gyro_bandwidth(struct inv_icm20689 * s, int level);

/** @brief Returns fullscale range of gyroscope in hardware
 *  @return the fullscale range
 */
uint8_t INV_EXPORT inv_icm20689_get_gyro_fullscale(struct inv_icm20689 * s);

/** @brief Returns bandwidth range of gyroscope in hardware
 *  @return the bandwidth range
 */
uint16_t INV_EXPORT inv_icm20689_get_gyro_bandwidth(struct inv_icm20689 * s);

/** @brief Get the available features according the base sensor device
 *  @param[in] in  	0: No advanced features supported
 *                  1: OIS sensor and FSYNC behavior supported (for icm20690 only)
 */
int INV_EXPORT inv_icm20689_is_advanced_features_supported(void);

/** @brief Set FSYNC bit location. Only supported in ICM20690.
 *  @param[in] in  	0: Disable FSYNC pin data to be sampled
 *                  1: Enable FSYNC pin data to be sampled at TEMP_OUT_L[0]
 */
int INV_EXPORT inv_icm20689_set_fsync_bit_location(struct inv_icm20689 * s, int bit_location);


#ifdef __cplusplus
}
#endif

#endif /* _INV_icm20689_SETUP_H_ */

/** @} */
