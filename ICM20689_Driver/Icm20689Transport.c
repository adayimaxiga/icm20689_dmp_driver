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

#include "icm20689Transport.h"
#include "icm20689Serif.h"
#include "icm20689Defs.h"
#include "icm20689.h"

int inv_icm20689_read_reg(struct inv_icm20689 * s, uint8_t reg,	uint8_t * buf, uint32_t len)
{
	return inv_icm20689_serif_read_reg(&s->serif, reg, buf, len);
}

int inv_icm20689_write_reg(struct inv_icm20689 * s, uint8_t reg, const uint8_t * buf, uint32_t len)
{
	return inv_icm20689_serif_write_reg(&s->serif, reg, buf, len);
}


// These registers are in sclk or sclk/mclk domains and
// can be written in LP mode or sleep state
static uint8_t check_reg_access_lp_disable(unsigned short reg)
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
	(void)reg;
	// All Athens registers are in sclk domain
	return 0;
#else
	switch(reg){
		case 0x19:
		case 0x1D:
		case 0x1E:
		case 0x37:
		case 0x38:
		case 0x6B:
		case 0x6C:
		case 0x72:
		case 0x73:
		case 0x74:
			return 0;
		default:
			break;
	}
	return 1;
#endif
}


/* the following functions are used for configuring the secondary devices */

int inv_icm20689_mems_write_reg(struct inv_icm20689 * s, uint16_t reg, uint32_t length, const uint8_t *data)
{
    int result = 0;
	uint32_t bytesWrite = 0;
    const uint8_t regOnly = (uint8_t)(reg & 0x7F);

    const uint8_t orig_power_state = inv_icm20689_get_chip_power_state(s);

	if((orig_power_state & CHIP_AWAKE) == 0) {
		// Wake up chip since it is asleep
		if(check_reg_access_lp_disable(reg))    // Check if register needs to be awake
			result = inv_icm20689_set_chip_power_state(s, CHIP_AWAKE, 1);
	}

	if((orig_power_state & CHIP_LP_ENABLE) != 0) {
		if(check_reg_access_lp_disable(reg))    // Check if register needs LP_EN to be disabled
			result |= inv_icm20689_set_chip_power_state(s, CHIP_LP_ENABLE, 0);   // Disable LP_EN
	}

	while (bytesWrite<length)
	{
		int thisLen = INV_MIN(MAX_SERIAL_WRITE, length-bytesWrite);

		result |= inv_icm20689_write_reg(s, regOnly+bytesWrite, &data[bytesWrite], thisLen);

		if (result)
			return result;

		bytesWrite += thisLen;
	}

    // Put the chip back to LP mode if it was enabled originally
	if((orig_power_state & CHIP_LP_ENABLE) != 0) {
		if(check_reg_access_lp_disable(reg))    // Check if register needs LP_EN to be disabled
			result |= inv_icm20689_set_chip_power_state(s, CHIP_LP_ENABLE, 1);   // Enable LP_EN
	}

    // Put the chip back to sleep if it was asleep originally
	if((orig_power_state & CHIP_AWAKE) == 0) {
		if(check_reg_access_lp_disable(reg))    // Check if register needs to be awake
			result |= inv_icm20689_set_chip_power_state(s, CHIP_AWAKE, 0);
	}

	return result;
}

int inv_icm20689_mems_write_reg_one(struct inv_icm20689 * s, uint16_t reg, uint8_t data)
{
	const uint8_t regOnly = (uint8_t)(reg & 0x7F);

	return inv_icm20689_write_reg(s, regOnly, &data, 1);
}

int inv_icm20689_mems_read_reg(struct inv_icm20689 * s, uint16_t reg, uint32_t length, uint8_t *data)
{
	int result = 0;
	uint32_t bytesRead = 0;
	const uint8_t regOnly = (uint8_t)(reg & 0x7F);

	while(bytesRead<length) {
		int thisLen = INV_MIN(MAX_SERIAL_READ, length-bytesRead);

		result |= inv_icm20689_read_reg(s, regOnly+bytesRead, &data[bytesRead], thisLen);

		if (result)
		return result;

		bytesRead += thisLen;
	}

	return result;
}

/**
*  @brief      Read data from a register in DMP memory 
*  @param[in]  DMP memory address
*  @param[in]  number of byte to be read
*  @param[in]  input data from the register
*  @return     0 if successful.
*/

int inv_icm20689_read_mems(struct inv_icm20689 * s, unsigned short reg, unsigned int length, unsigned char *data)
{
	int result=0;
	unsigned int bytesWritten = 0;
	unsigned int thisLen;
	unsigned char i, dat[MAX_SERIAL_READ] = {0};
	unsigned char lBankSelected;
	unsigned char lStartAddrSelected;
	unsigned char power_state;

	if(!data)
		return -1;

    power_state = inv_icm20689_get_chip_power_state(s);
	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20689_set_chip_power_state(s, CHIP_AWAKE, 1);

	lBankSelected = (reg >> 8);
	if (lBankSelected != s->lLastBankSelected) {
		result |= inv_icm20689_write_reg(s, REG_MEM_BANK_SEL, &lBankSelected, 1);
		if (result)
			return result;
		s->lLastBankSelected = lBankSelected;
	}

	while (bytesWritten < length) {
		lStartAddrSelected = (reg & 0xff);
		/* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		   Contents are changed after read or write of the selected memory.
		   This register must be written prior to each access to initialize the register to the proper starting address.
		   The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
		result |= inv_icm20689_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
		if (result)
			return result;
		
		thisLen = INV_MIN(MAX_SERIAL_READ, length-bytesWritten);
		/* Write data */
		if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
			result |= inv_icm20689_read_reg(s, REG_MEM_R_W, &dat[bytesWritten], thisLen);
		} else {
			result |= inv_icm20689_read_reg(s, REG_MEM_R_W, &data[bytesWritten], thisLen);
		}
		if (result)
			return result;
		
		bytesWritten += thisLen;
		reg += thisLen;
	}

	if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
		for (i=0; i< length; i++) {
			*data= dat[i];
			 data++;
		}
	}

	return result;
}

/**
*  @brief       Write data to a register in DMP memory 
*  @param[in]   DMP memory address
*  @param[in]   number of byte to be written
*  @param[out]  output data from the register
*  @return     0 if successful.
*/

int inv_icm20689_write_mems(struct inv_icm20689 * s, unsigned short reg, unsigned int length, const unsigned char *data)
{
    int result=0;
    unsigned int bytesWritten = 0;
    unsigned int thisLen;
    unsigned char lBankSelected;
    unsigned char lStartAddrSelected;
	unsigned char power_state;

    if(!data)
        return -1;

    power_state = inv_icm20689_get_chip_power_state(s);
    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_icm20689_set_chip_power_state(s, CHIP_AWAKE, 1);

    lBankSelected = (reg >> 8);
    if (lBankSelected != s->lLastBankSelected) {
            result |= inv_icm20689_write_reg(s, REG_MEM_BANK_SEL, &lBankSelected, 1);
            if (result)
                    return result;
            s->lLastBankSelected = lBankSelected;
    }

    while (bytesWritten < length) {
        lStartAddrSelected = (reg & 0xff);
        /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
        result |= inv_icm20689_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
        if (result)
            return result;
        
        thisLen = INV_MIN(MAX_SERIAL_WRITE, length-bytesWritten);
        
        /* Write data */ 
        result |= inv_icm20689_write_reg(s, REG_MEM_R_W, &data[bytesWritten], thisLen);
        if (result)
            return result;
        
        bytesWritten += thisLen;
        reg += thisLen;
    }

    return result;
}


