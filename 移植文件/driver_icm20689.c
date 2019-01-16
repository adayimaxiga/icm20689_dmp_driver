#include "driver_icm20689.h"
#include "spi.h"


//-----------------------------------
#define ICM_20689_ENABLE   PAout(4)=0
#define ICM_20689_DISABLE  PAout(4)=1
//----------------------------------

/*				REGISTER MAP			*/
#define SELF_TEST_X_GYRO  0x00
#define SELF_TEST_Y_GYRO  0x01 
#define SELF_TEST_Z_GYRO  0x02

#define SELF_TEST_X_ACCEL 0x0D	 
#define SELF_TEST_Y_ACCEL 0x0E	 
#define SELF_TEST_Z_ACCEL 0x0F	

#define XG_OFFS_USRH      0x13	
#define XG_OFFS_USRL      0x14
#define YG_OFFS_USRH      0x15
#define YG_OFFS_USRL      0x16
#define ZG_OFFS_USRH      0x17
#define ZG_OFFS_USRL      0x18

#define SMPLRT_DIV        0x19

#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG_2    0x1D

#define LOW_POWER_MODE		0x1E

#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器

#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

#define PWR_MGMT_1        0x6B
#define PWR_MGMT_2        0x6C
#define WHO_AM_I          0x75


static uint8_t ICM20689_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	ICM_20689_ENABLE;
	status = HAL_SPI_Transmit(&hspi1, &reg, 1, 0xFFFF);
	status = HAL_SPI_Transmit(&hspi1, &value, 1, 0xFFFF);
	ICM_20689_DISABLE;
	return(status);
}

static uint8_t ICM20689_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	ICM_20689_ENABLE;	
	reg = reg|0x80;
	HAL_SPI_Transmit(&hspi1, &reg, 1, 0xFFFF);	 	
 	HAL_SPI_Receive(&hspi1, &reg_val, 1, 0xFFFF);	
	ICM_20689_DISABLE;															
	return(reg_val);
}

void icm20689_reset()
{
	ICM20689_Write_Reg(PWR_MGMT_1,0X80);	//复位
	HAL_Delay(100);
	ICM20689_Write_Reg(PWR_MGMT_1, 0x00);	//解除休眠状态
}
//不用DMP初始化版本
uint8_t Init_ICM20689(void)
{	
	if(ICM20689_Read_Reg(WHO_AM_I)==0x98)
	{
	}
	else
	{
		return 1;
	}
	
	ICM20689_Write_Reg(PWR_MGMT_1,0X80);	//复位
	HAL_Delay(100);
	ICM20689_Write_Reg(PWR_MGMT_1, 0x00);	//解除休眠状态
	
	ICM20689_Write_Reg(GYRO_CONFIG,3<<3);//设置陀螺仪满量程范围
	ICM20689_Write_Reg(ACCEL_CONFIG,0<<3);//加速度计工作范围 2G
	ICM20689_Write_Reg(ACCEL_CONFIG_2, 0x08);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）	
	
	ICM20689_Write_Reg(SMPLRT_DIV,19);
	ICM20689_Write_Reg(CONFIG,4);					//低通滤波频率，典型值：0x07(3600Hz)此寄存器内决定Internal_Sample_Rate==8K
	
	ICM20689_Write_Reg(MPU_INT_EN_REG,0X00);	//关闭所有中断
	ICM20689_Write_Reg(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	
	ICM20689_Write_Reg(PWR_MGMT_1,0X01);	//设置CLKSEL,PLL X轴为参考
	ICM20689_Write_Reg(PWR_MGMT_2,0X00);	//加速度与陀螺仪都工作
	

	return 0;
}
//注意单位-----使用dmp模式时加速度量程一定是 +/- 4g
unsigned char MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    unsigned char buf[6],res;  
	res=MPU_Read_Len(0x00,ACCEL_XOUT_H,6,buf);
	if(res==0)
	{
		*ax=((unsigned short int)buf[0]<<8)|buf[1];  
		*ay=((unsigned short int)buf[2]<<8)|buf[3];  
		*az=((unsigned short int)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//注意单位-----使用dmp模式时陀螺仪量程一定是 +/- 2000dps
unsigned char MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    unsigned char buf[6],res;  
	res=MPU_Read_Len(0x00,GYRO_XOUT_H,6,buf);
	if(res==0)
	{
		*gx=((unsigned short int)buf[0]<<8)|buf[1];  
		*gy=((unsigned short int)buf[2]<<8)|buf[3];  
		*gz=((unsigned short int)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
	uint8_t status;
	ICM_20689_ENABLE;
	status = HAL_SPI_Transmit(&hspi1, &reg, 1, 0xFFFF);
	status = HAL_SPI_Transmit(&hspi1, buf, len, 0xFFFF);
	ICM_20689_DISABLE;
	
	return status;	
} 

unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{ 
	uint8_t status;
	ICM_20689_ENABLE;	
	reg = reg|0x80;
	status=HAL_SPI_Transmit(&hspi1, &reg, 1, 0xFFFF);	 	
 	status=HAL_SPI_Receive(&hspi1, buf, len, 0xFFFF);	
	ICM_20689_DISABLE;						
	return status;	
}


//**********************************************************************************************************************************************************
static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)context;
	return MPU_Read_Len(0x00, reg, rlen, rbuffer);
}
static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)context;
	return MPU_Write_Len(0x00, reg, wlen, (uint8_t*)wbuffer);

}

void dmp_setup(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len));
float* icm20689_data_poll(void);

//用DMP初始化版本
void icm20689_dmp_setup()
{
	icm20689_reset();								//必须先reset
	dmp_setup(idd_io_hal_read_reg,idd_io_hal_write_reg);
}
int count=0;
//读取  0表示成功  1表示失败
char get_dmp_data(float* yaw,float* pitch, float* roll)
{
	float * point =icm20689_data_poll();
	
	if(point)
	{
		*yaw = *point;
		*pitch = *(point+1);
		*roll = *(point+2);
		count++;
		return 0;
	}
	return 1;
}



