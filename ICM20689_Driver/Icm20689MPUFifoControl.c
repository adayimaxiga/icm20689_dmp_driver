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
#include "icm20689MPUFifoControl.h"

#include "icm20689Defs.h"
//#include "icm20689DataBaseControl.h"
//#include "icm20689DataConverter.h"


struct inv_fifo_decoded_t fd;

static void inv_decode_3_16bit_elements(short *out_data, const unsigned char *in_data);
static void inv_decode_3_32bit_elements(long *out_data, const unsigned char *in_data);


int inv_icm20689_identify_interrupt(struct inv_icm20689 * s, short *int_read)
{
	unsigned char int_status;
    int result=0 ;
    
    if(int_read)
        *int_read = 0;
    
    result = inv_icm20689_mems_read_reg(s, REG_INT_STATUS, 1, &int_status);
    if(int_read)
        *int_read = int_status;

    result = inv_icm20689_mems_read_reg(s, REG_DMP_INT_STATUS, 1, &int_status); // DMP_INT_STATUS
	if(int_read)
		*int_read |= (int_status << 8);
    
    /*if(wake_on_motion_enabled) {
        result = inv_icm20689_read_mems_reg(s, REG_INT_STATUS, 1, &int_status);//INT_STATUS
        if(result)
            return result;
        *int_read |= reg_data[1];
    }*/
    /*
     * We do not need to handle FIFO overflow here. 
     * When we read FIFO_SIZE we can determine if FIFO overflow has occured.
     */
    //result = inv_icm20689_read_mems_reg(s, 0x1B, 1, &int_status);
    
	return result;
}

/**
* @internal
* @brief   Get the length from the fifo
*
* @param[out] len amount of data currently stored in the fifo.
*
* @return MPU_SUCCESS or non-zero error code.
**/
static int dmp_get_fifo_length(struct inv_icm20689 * s, uint_fast16_t * len )
{
	unsigned char fifoBuf[2];
	int result = 0;
    
	if (NULL == len)
		return -1;
    
	/*---- read the 2 'count' registers and
	burst read the data from the FIFO ----*/
	result = inv_icm20689_mems_read_reg(s, REG_FIFO_COUNT_H, 2, fifoBuf);
	if (result) 
	{
		s->fifo_info.fifoError = -1;
		*len = 0;
		return result;
	}
    
	*len = (uint_fast16_t) (fifoBuf[0] << 8);
	*len += (uint_fast16_t) (fifoBuf[1]);

	return result;
}

/**
*  @internal
*  @brief  Clears the FIFO status and its content.
*  @note   Halt the DMP writing into the FIFO for the time
*          needed to reset the FIFO.
*  @return MPU_SUCCESS if successful, a non-zero error code otherwise.
*/
static int dmp_reset_fifo(struct inv_icm20689 * s)
{
    uint_fast16_t len = HARDWARE_FIFO_SIZE;
	unsigned char tries = 0;
	int result = 0;
	unsigned char data = 0;
    
	while (len != 0 && tries < 6) 
	{ 
		s->base_state.user_ctrl &= (~BIT_FIFO_EN);
		s->base_state.user_ctrl &= (~BIT_DMP_EN);
		result |= inv_icm20689_mems_write_reg(s, REG_USER_CTRL, 1, &s->base_state.user_ctrl);
		inv_icm20689_sleep(5);
		
		data = BIT_FIFO_RST;
        result |= inv_icm20689_mems_write_reg(s, REG_USER_CTRL, 1, &data);
		data = BIT_DMP_RST;
        result |= inv_icm20689_mems_write_reg(s, REG_USER_CTRL, 1, &data);
        inv_icm20689_sleep(2);
        
		// Reset overflow flag
		s->fifo_info.fifo_overflow = 0;
        
		result |= dmp_get_fifo_length(s, &len);
		if (result) 
			return result;
        
		tries++;
	}
    
	s->base_state.user_ctrl |= BIT_FIFO_EN;
	s->base_state.user_ctrl |= BIT_DMP_EN;
	result |= inv_icm20689_mems_write_reg(s, REG_USER_CTRL, 1, &s->base_state.user_ctrl);
    
	return result;
}

/**
*  @internal
*  @brief  Read data from the fifo
*
*  @param[out] data Location to store the date read from the fifo
*  @param[in] len   Amount of data to read out of the fifo
*
*  @return MPU_SUCCESS or non-zero error code
**/
static int dmp_read_fifo(struct inv_icm20689 * s, unsigned char *data, uint_fast16_t len)
{
	int result;
    uint_fast16_t bytesRead = 0;

    while (bytesRead<len) 
    {
        unsigned short thisLen = INV_MIN(MAX_SERIAL_READ, len-bytesRead);
        
        result = inv_icm20689_mems_read_reg(s, REG_FIFO_R_W, thisLen, &data[bytesRead]);
        if (result)
		{
			dmp_reset_fifo(s);
			s->fifo_info.fifoError = -1;
			return result;
		}
        
        bytesRead += thisLen;
    }

	return result;
}

/**
*  @internal
*  @brief  used to get the FIFO data.
*  @param  length
*              Max number of bytes to read from the FIFO that buffer is still able to sustain.
*  @param  buffer Reads up to length into the buffer.
*
*  @return number of bytes of read.
**/
static uint_fast16_t dmp_get_fifo_all(struct inv_icm20689 * s, uint_fast16_t length, unsigned char *buffer, int *reset)
{
	int result;
	uint_fast16_t in_fifo;
    
	if(reset)
		*reset = 0;
   
	result = dmp_get_fifo_length(s, &in_fifo);
	if (result) {
		s->fifo_info.fifoError = result;
		return 0;
	}
    
	// Nothing to read
	if (in_fifo == 0)
		return 0;
    
	/* Check if buffer is able to be filled in with in_fifo bytes */
	if (in_fifo > length) {
		dmp_reset_fifo(s);
		s->fifo_info.fifoError = -1;
		if(reset)
			*reset = 1;
		return 0;
	}

	result = dmp_read_fifo(s, buffer, in_fifo);
	if (result) {
		s->fifo_info.fifoError = result;
		return 0;
	}
	return in_fifo;
}

/** Determines the packet size by decoding the header. Both header and header2 are set. header2 is set to zero
*   if it doesn't exist. sample_cnt_array is filled in if not null with number of samples expected for each sensor
*/
static uint_fast16_t get_packet_size_and_samplecnt(unsigned char *data, unsigned short *header, unsigned short *header2, unsigned short * sample_cnt_array)
{
	int sz = HEADER_SZ; // 2 for header
    
	*header = (((unsigned short)data[0])<<8) | data[1];

	if (*header & ACCEL_SET) {
		sz += ACCEL_DATA_SZ;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_ACCELEROMETER]++;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_RAW_ACCELEROMETER]++;
	}
    
	if (*header & GYRO_SET) {
		sz += GYRO_DATA_SZ;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED]++;
		sz += GYRO_BIAS_DATA_SZ;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_GYROSCOPE]++;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_RAW_GYROSCOPE]++;
	}
 

	if (*header & QUAT6_SET) {
		sz += QUAT6_DATA_SZ;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_GAME_ROTATION_VECTOR]++;
	}


	if (*header & PQUAT6_SET) 
		sz += PQUAT6_DATA_SZ;
    
    
	if (*header & GYRO_CALIBR_SET) {
		sz += GYRO_CALIBR_DATA_SZ;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_GYROSCOPE]++;
	}

	if (*header & PED_STEPDET_SET) {
		sz += PED_STEPDET_TIMESTAMP_SZ;
		if (sample_cnt_array)
			sample_cnt_array[ANDROID_SENSOR_STEP_DETECTOR]++;
	}

	if (*header & HEADER2_SET) {
		*header2 = (((unsigned short)data[2])<<8) | data[3];
		sz += HEADER2_SZ;
	} else {
		*header2 = 0;
	}
    
	if (*header2 & ACCEL_ACCURACY_SET) {
		sz += ACCEL_ACCURACY_SZ;
	}
	if (*header2 & GYRO_ACCURACY_SET) {
		sz += GYRO_ACCURACY_SZ;
	}

	return sz;
}

static int check_fifo_decoded_headers(unsigned short header, unsigned short header2)
{
	unsigned short header_bit_mask = 0;
	unsigned short header2_bit_mask = 0;
	
	// at least 1 bit must be set
	if (header == 0)
		return -1;
	
	header_bit_mask |= ACCEL_SET;
	header_bit_mask |= GYRO_SET;
	header_bit_mask |= QUAT6_SET;
	header_bit_mask |= PQUAT6_SET;
	header_bit_mask |= GYRO_CALIBR_SET;
	header_bit_mask |= PED_STEPDET_SET;
	header_bit_mask |= HEADER2_SET;
	
	if (header & ~header_bit_mask)
		return -1;
	
	// at least 1 bit must be set if header 2 is set
	if (header & HEADER2_SET) {
		header2_bit_mask |= ACCEL_ACCURACY_SET;
		header2_bit_mask |= GYRO_ACCURACY_SET;
		if (header2 == 0)
			return -1;
		if (header2 & ~header2_bit_mask)
			return -1;
	}

    return 0;
}

/** Software FIFO, mirror of DMP HW FIFO, hence of max HARDWARE_FIFO_SIZE */
static unsigned char fifo_data[HARDWARE_FIFO_SIZE];
    
/** Determine number of samples present in SW FIFO fifo_data containing fifo_size bytes to be analyzed. Total number
* of samples filled in total_sample_cnt, number of samples per sensor filled in sample_cnt_array array
*/
static int extract_sample_cnt(struct inv_icm20689 * s, int fifo_size, unsigned short * total_sample_cnt, unsigned short * sample_cnt_array)
{
	// Next SW FIFO index to be parsed
	int fifo_idx = 0;
	
	while (fifo_idx < fifo_size) {
		unsigned short header;
		unsigned short header2;
		int need_sz = get_packet_size_and_samplecnt(&fifo_data[fifo_idx], &header, &header2, sample_cnt_array);
		
		// Guarantee there is a full packet before continuing to decode the FIFO packet
		if (fifo_size-fifo_idx < need_sz)
			goto endSuccess;
		
		// Decode any error
		if (check_fifo_decoded_headers(header, header2)) {
			// in that case, stop processing, we might have overflowed so following bytes are non sense
			dmp_reset_fifo(s);
			return -1;
		}
		
		fifo_idx += need_sz;
		
		// One sample found, increment total sample counter
		(*total_sample_cnt)++;
	}

endSuccess:
#if defined MEMS_AUGMENTED_SENSORS
	// Augmented sensors are not part of DMP FIFO, they are computed by DMP driver based on GRV or RV presence in DMP FIFO
	// So their sample counts must rely on GRV and RV sample counts
	if (sample_cnt_array) {
		sample_cnt_array[ANDROID_SENSOR_GRAVITY] += sample_cnt_array[ANDROID_SENSOR_GAME_ROTATION_VECTOR];
		sample_cnt_array[ANDROID_SENSOR_LINEAR_ACCELERATION] += sample_cnt_array[ANDROID_SENSOR_GAME_ROTATION_VECTOR];
		sample_cnt_array[ANDROID_SENSOR_ORIENTATION] += sample_cnt_array[ANDROID_SENSOR_ROTATION_VECTOR];
	}
#endif

	return 0;
}

int inv_icm20689_fifo_swmirror(struct inv_icm20689 * s, int *fifo_sw_size, unsigned short * total_sample_cnt, unsigned short * sample_cnt_array)
{
	int reset=0; 

	*total_sample_cnt = 0;

	// Mirror HW FIFO into local SW FIFO, taking into account remaining *fifo_sw_size bytes still present in SW FIFO
	if (*fifo_sw_size < HARDWARE_FIFO_SIZE ) {
		*fifo_sw_size += dmp_get_fifo_all(s, (HARDWARE_FIFO_SIZE - *fifo_sw_size),&fifo_data[*fifo_sw_size],&reset);

		if (reset)
			goto error;
	}

	// SW FIFO is mirror, we can now parse it to extract total number of samples and number of samples per sensor
	if (extract_sample_cnt(s, *fifo_sw_size, total_sample_cnt, sample_cnt_array))
			goto error;

	return MPU_SUCCESS;
	
error:
	*fifo_sw_size = 0;
	return -1;
	
}

int inv_icm20689_fifo_pop(struct inv_icm20689 * s, unsigned short *user_header, unsigned short *user_header2, int *fifo_sw_size)  
{
	int need_sz=0; // size in bytes of packet to be analyzed from FIFO
	unsigned char *fifo_ptr = fifo_data; // pointer to next byte in SW FIFO to be parsed
    
	if (*fifo_sw_size > 3) {
		// extract headers and number of bytes requested by next sample present in FIFO
		need_sz = get_packet_size_and_samplecnt(fifo_data, &fd.header, &fd.header2, 0);

		// Guarantee there is a full packet before continuing to decode the FIFO packet
		if (*fifo_sw_size < need_sz) {
		    return s->fifo_info.fifoError;
		}

		fifo_ptr += HEADER_SZ;        
		if (fd.header & HEADER2_SET)
			fifo_ptr += HEADER2_SZ;        

		// extract payload data from SW FIFO
		fifo_ptr += inv_icm20689_inv_decode_one_fifo_packet(s, &fd, fifo_ptr);        

		// remove first need_sz bytes from SW FIFO
		*fifo_sw_size -= need_sz;
		if(*fifo_sw_size)
			memmove(fifo_data, &fifo_data[need_sz], *fifo_sw_size);// Data left in FIFO

		*user_header = fd.header;
		*user_header2 = fd.header2;
	}

	return MPU_SUCCESS;
}
#if 0
int inv_icm20689_dmp_process_fifo(struct inv_icm20689 * s, int *left_in_fifo, unsigned short *user_header, unsigned short *user_header2, long *time_stamp)  
{
    int result = MPU_SUCCESS;
    int reset=0; 
    int need_sz=0;
    unsigned char *fifo_ptr = fifo_data;

    long long ts=0;

    if(!left_in_fifo)
        return -1;
    
    if (*left_in_fifo < HARDWARE_FIFO_SIZE ) 
    {
        *left_in_fifo += dmp_get_fifo_all(s, (HARDWARE_FIFO_SIZE - *left_in_fifo),&fifo_data[*left_in_fifo],&reset);
        //sprintf(test_str, "Left in FIFO: %d\r\n",*left_in_fifo);
        //print_command_console(test_str);
        if (reset) 
        {
            *left_in_fifo = 0;
            return -1;
        }
    }
    
    if (*left_in_fifo > 3) {
	// no need to extract number of sample per sensor for current function, so provide 0 as last parameter
        need_sz = get_packet_size_and_samplecnt(fifo_data, &fd.header, &fd.header2, 0);
        
        // Guarantee there is a full packet before continuing to decode the FIFO packet
        if (*left_in_fifo < need_sz) {
            result = s->fifo_info.fifoError;
            s->fifo_info.fifoError = 0;
            return result;
        }

        if(user_header)
            *user_header = fd.header;
        
        if(user_header2)
            *user_header2 = fd.header2;
        
        if (check_fifo_decoded_headers(fd.header, fd.header2)) { 
            // Decode error
            dmp_reset_fifo(s);
            *left_in_fifo = 0;
            return -1;
        }
        
        fifo_ptr += HEADER_SZ;
        
        if (fd.header & HEADER2_SET)
            fifo_ptr += HEADER2_SZ;        
        
        //time stamp 
        ts = inv_icm20689_get_tick_count();
        
        fifo_ptr += inv_icm20689_inv_decode_one_fifo_packet(s, &fd, fifo_ptr);

        if(time_stamp)
            *time_stamp = ts;
        
        /* Parse the data in the fifo, in the order of the data control register, starting with the MSB(accel)
        */        
        
        *left_in_fifo -= need_sz;
        if (*left_in_fifo) 
            memmove(fifo_data, &fifo_data[need_sz], *left_in_fifo);// Data left in FIFO
    }

    return result;
}
#endif

static void inv_decode_3_32bit_elements(long *out_data, const unsigned char *in_data)
{
    out_data[0] = ((long)(0xff & in_data[0]) << 24) | ((long)(0xff & in_data[1]) << 16) | ((long)(0xff & in_data[2]) << 8) | (0xff & in_data[3]);
    out_data[1] = ((long)(0xff & in_data[4]) << 24) | ((long)(0xff & in_data[5]) << 16) | ((long)(0xff & in_data[6]) << 8) | (0xff & in_data[7]);
    out_data[2] = ((long)(0xff & in_data[8]) << 24) | ((long)(0xff & in_data[9]) << 16) | ((long)(0xff & in_data[10]) << 8) | (0xff & in_data[11]);
}

static void inv_decode_3_16bit_elements(short *out_data, const unsigned char *in_data)
{
    out_data[0] = ((short)(0xff & in_data[0]) << 8) | (0xff & in_data[1]);
    out_data[1] = ((short)(0xff & in_data[2]) << 8) | (0xff & in_data[3]);
    out_data[2] = ((short)(0xff & in_data[4]) << 8) | (0xff & in_data[5]);
}

/** Decodes one packet of data from FIFO
* @param[in] fd Structure to be filled out with data. Assumes header and header2 are already set inside.
* @param[in] fifo_ptr FIFO data, points to just after any header information
* @return Returns the number of bytes consumed in FIFO data.
*/
int inv_icm20689_inv_decode_one_fifo_packet(struct inv_icm20689 * s, struct inv_fifo_decoded_t *fd, const unsigned char *fifo_ptr)
{
    const unsigned char *fifo_ptr_start = fifo_ptr;  
    if (fd->header & ACCEL_SET) {
        // do not cast data here, do that when you use it
        inv_decode_3_16bit_elements(fd->accel_s, fifo_ptr);
        fd->accel[0] = fd->accel_s[0] << 15;
        fd->accel[1] = fd->accel_s[1] << 15;
        fd->accel[2] = fd->accel_s[2] << 15;
        fifo_ptr += ACCEL_DATA_SZ;
    }

    if (fd->header & GYRO_SET) {
        inv_decode_3_16bit_elements(fd->gyro, fifo_ptr);
        fifo_ptr += GYRO_DATA_SZ;
        inv_decode_3_32bit_elements(fd->gyro_bias, fifo_ptr);
        fifo_ptr += GYRO_BIAS_DATA_SZ;
    }

    if (fd->header & QUAT6_SET) {
		// The header got pushed twice for some reason, hence fifo_ptr+2
        inv_decode_3_32bit_elements(fd->dmp_3e_6quat, fifo_ptr+2);
        fifo_ptr += QUAT6_DATA_SZ;
    }

    if (fd->header & GYRO_CALIBR_SET) {
        inv_decode_3_32bit_elements(fd->gyro_calibr, fifo_ptr);
        fifo_ptr += GYRO_CALIBR_DATA_SZ;
    }

    if (fd->header & PED_STEPDET_SET) {
        fd->ped_step_det_ts = ((0xff & fifo_ptr[2]) << 24) | ((0xff & fifo_ptr[3]) << 16) | ((0xff & fifo_ptr[4]) << 8) | (0xff & fifo_ptr[5]);
        fifo_ptr += PED_STEPDET_TIMESTAMP_SZ;
    }
    if (fd->header2 & ACCEL_ACCURACY_SET) {
        fd->accel_accuracy = ((0xff & fifo_ptr[0]) << 8) | (0xff & fifo_ptr[1]);
        fifo_ptr += ACCEL_ACCURACY_SZ;
    }
        
    if (fd->header2 & GYRO_ACCURACY_SET) {
        fd->gyro_accuracy = ((0xff & fifo_ptr[0]) << 8) | (0xff & fifo_ptr[1]);
        fifo_ptr += GYRO_ACCURACY_SZ;
    }
 
    fd->new_data = 1; // Record a new data set

    return fifo_ptr-fifo_ptr_start;
}

int inv_icm20689_dmp_get_accel(long acl[3])
{
    if(!acl) return -1;
    memcpy( acl, fd.accel, 3*sizeof(long));
    return MPU_SUCCESS;
} 

int inv_icm20689_dmp_get_raw_gyro(short raw_gyro[3])
{
    if(!raw_gyro) return -1;
    raw_gyro[0] = fd.gyro[0];
    raw_gyro[1] = fd.gyro[1];
    raw_gyro[2] = fd.gyro[2];
    return MPU_SUCCESS;
}

int inv_icm20689_dmp_get_gyro_bias(long gyro_bias[3])
{
    if(!gyro_bias) return -1;
    memcpy(gyro_bias, fd.gyro_bias, 3*sizeof(long));
    return MPU_SUCCESS;
}

int inv_mems_dmp_get_calibrated_gyro(signed long calibratedData[3], short raw[3], signed long bias[3])
{
    if(!calibratedData) return -1;
    if(!raw) return -1;
    if(!bias) return -1;
    signed long scaled_bias[3];

    for (int i =0; i<3; i++){
      scaled_bias[i] = bias[i] * ((1<< 15)/2859);
      calibratedData[i] = (raw[i]<<15) - scaled_bias[i];
    }

    return MPU_SUCCESS;
}

int inv_icm20689_dmp_get_calibrated_gyro(long cal_gyro[3])
{
    if(!cal_gyro) return -1;
    memcpy(cal_gyro, fd.gyro_calibr, 3*sizeof(long));
    return MPU_SUCCESS;
}

int inv_icm20689_dmp_get_6quaternion(long quat[3])
{
    if(!quat) return -1;
    memcpy( quat, fd.dmp_3e_6quat, sizeof(fd.dmp_3e_6quat));            
    return MPU_SUCCESS;
}

int inv_icm20689_dmp_get_9quaternion(long quat[3])
{
    if(!quat) return -1;
    memcpy( quat, fd.dmp_3e_9quat, sizeof(fd.dmp_3e_9quat));            
    return MPU_SUCCESS;
}


int inv_icm20689_dmp_get_raw_compass(long raw_compass[3])
{
    if(!raw_compass) return -1;
    memcpy( raw_compass, fd.compass, 3*sizeof(long)); 
    return MPU_SUCCESS;
}

int inv_icm20689_dmp_get_calibrated_compass(long cal_compass[3])
{
    if(!cal_compass) return -1;
    memcpy( cal_compass, fd.cpass_calibr, 3*sizeof(long));  
    return MPU_SUCCESS;
}

int inv_icm20689_dmp_get_bac_state(uint16_t *bac_state)
{
	if(!bac_state) return -1;
	*bac_state = fd.bac_state;
	return 0;
}

int inv_icm20689_dmp_get_bac_ts(long *bac_ts)
{
	if(!bac_ts) return -1;
	*bac_ts = fd.bac_ts;
	return 0;
}

int inv_icm20689_dmp_get_flip_pickup_state(uint16_t *flip_pickup)
{
	if(!flip_pickup) return -1;
	*flip_pickup = fd.flip_pickup;
	return 0;
}

/** Returns accuracy of accel.
 * @return Accuracy of accel with 0 being not accurate, and 3 being most accurate.
*/
int inv_icm20689_get_accel_accuracy(void)
{
	return fd.accel_accuracy;
}

/** Returns accuracy of gyro.
 * @return Accuracy of gyro with 0 being not accurate, and 3 being most accurate.
*/
int inv_icm20689_get_gyro_accuracy(void)
{
	return fd.gyro_accuracy;
}

