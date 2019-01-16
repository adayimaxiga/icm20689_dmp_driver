#ifndef ICM20689_H
#define ICM20689_H
#include "sys.h"

uint8_t Init_ICM20689(void);
unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf);
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf);

void icm20689_dmp_setup(void);
char get_dmp_data(float* yaw,float* pitch, float* roll);
unsigned char MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
unsigned char MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif


