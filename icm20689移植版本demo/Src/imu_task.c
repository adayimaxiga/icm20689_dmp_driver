#include "imu_task.h"
#include "cmsis_os.h"
#include "driver_icm20689.h"

float yaw,pitch,roll;
s16 ax,ay,az;
s16 gx,gy,gz;
void IMUTask(void const * argument)
{
	
	
	
	while(1)
	{
		get_dmp_data(&yaw,&pitch,&roll);
		MPU_Get_Gyroscope(&gx,&gy,&gz);
		MPU_Get_Accelerometer(&ax,&ay,&az);
		osDelay(1);
	}
}

