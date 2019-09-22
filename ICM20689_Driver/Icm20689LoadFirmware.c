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

#include "icm20689.h"
#include "icm20689LoadFirmware.h"
#include "icm20689Defs.h"
#include "icm20689Dmp3Driver.h"
//#include "icm20689DataBaseDriver.h"

int inv_icm20689_firmware_load(struct inv_icm20689 * s, const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{ 
    int write_size;
    int result;
    unsigned short memaddr;
	unsigned short keyaddr;
    const unsigned char *data;
    unsigned short size;
    unsigned char data_cmp[MAX_SERIAL_READ+1];
    unsigned char data_key[MAX_SERIAL_READ+1];
    int flag = 0;
	unsigned char whoami;

	if(s->base_state.firmware_loaded)
		return 0;

	inv_icm20689_get_whoami(s, &whoami);	
	dmp_icm20689_get_sw_key_addr(s, &keyaddr);
		
    // Write DMP memory
    data = data_start;
    size = size_start;
    memaddr = load_addr;
    while (size > 0) {
        write_size = INV_MIN(size, MAX_SERIAL_WRITE);			
        if ((memaddr & 0xff) + write_size > 0x100) {
            // Moved across a bank
            write_size = (memaddr & 0xff) + write_size - 0x100;
        }
		
		if( (memaddr<=keyaddr) && ((memaddr+write_size)>=keyaddr) ) {
			memcpy(data_key, data, write_size);
			data_key[keyaddr-memaddr+1]=whoami;
	        result = inv_icm20689_write_mems(s, memaddr, write_size, data_key);
		} else {		
	        result = inv_icm20689_write_mems(s, memaddr, write_size, (unsigned char *)data);
		}
        if (result)  
            return result;
        data += write_size;
        size -= write_size;
        memaddr += write_size;
    }

    // Verify DMP memory

    data = data_start;
    size = size_start;
    memaddr = load_addr;
    while (size > 0) {
        write_size = INV_MIN(size, MAX_SERIAL_READ);
        if ((memaddr & 0xff) + write_size > 0x100) {
            // Moved across a bank
            write_size = (memaddr & 0xff) + write_size - 0x100;
        }
        result = inv_icm20689_read_mems(s, memaddr, write_size, data_cmp);
        if (result)
            flag++; // Error, DMP not written correctly
		if( (memaddr<=keyaddr) && ((memaddr+write_size)>=keyaddr) ) {
			memcpy(data_key, data, write_size);
			data_key[keyaddr-memaddr+1]=whoami;
	        if (memcmp(data_cmp, data_key, write_size))
				return -1;
		} else {
			if (memcmp(data_cmp, data, write_size))
				return -1;
		}
        data += write_size;
        size -= write_size;
        memaddr += write_size;
    }

    return 0;
}

