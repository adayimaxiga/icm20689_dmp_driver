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
/******************************************************************************/
/* Example configuration                                                      */
/******************************************************************************/
#ifndef SENSOR_H
#define SENSOR_H
#include "sys.h"
#include "icm20689.h"




#define DATA_ACCURACY_MASK    ((uint32_t)0x7)

#define DEFAULT_ODR_US    20000     // 20 ms sample spacing = 50 HZ
#define MIN_ODR_US        5000		// 5 ms sample spacing = 200 HZ
#define MAX_ODR_US        1000000	// 1000 ms sample spacing = 1 HZ

#define DEFAULT_PRESSURE_ODR_MS    25    // 25ms = 40Hz
#define MIN_PRESSURE_ODR_MS        25    // 25ms = 40Hz
#define MAX_PRESSURE_ODR_MS        1000  // 1000ms = 1HZ


extern int sensor_control(int enable);
extern int sensor_configure_odr(uint32_t odr_us);

float* icm20689_data_poll(void);

extern int icm20689_sensor_setup(void);
extern int icm20689_sensor_configuration(void);

uint64_t InvEMDFrontEnd_getTimestampUs(void);

extern enum inv_icm20689_sensor idd_sensortype_conversion(int sensor);

extern uint32_t period_us;

extern inv_icm20689_t icm_device;

extern uint32_t user_enabled_sensor_mask;

#endif


