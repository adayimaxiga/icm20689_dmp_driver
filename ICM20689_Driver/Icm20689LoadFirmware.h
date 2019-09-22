/*
* ________________________________________________________________________________________________________
* Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/

#ifndef _INV_icm20689_LOAD_FIRMWARE_H_
#define _INV_icm20689_LOAD_FIRMWARE_H_

/** @defgroup	icm20689_load_firmware	load_firmware
    @ingroup 	SmartSensor_driver
    @{
*/
#ifdef __cplusplus
extern "C"
{
#endif

/* forward declaration */
struct inv_icm20689;

/** @brief Loads the DMP firmware from SRAM
* @param[in] data  pointer where the image 
* @param[in] size  size if the image
* @param[in] load_addr  address to loading the image
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20689_firmware_load(struct inv_icm20689 * s, const unsigned char *data, unsigned short size, unsigned short load_addr);

#ifdef __cplusplus
}
#endif
#endif /* _INV_icm20689_LOAD_FIRMWARE_H_ */

/** @} */
