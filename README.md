ICM20689 DMP Driver
===

invensense公司新一代陀螺仪ICM20689网上没有驱动，官方提供一个ICM20789版本在G55板上的demo，
但是非常复杂难用，我基于这个版本删减并修改出来一个在STM32上方便使用的驱动代码。

## 平台参数

Chip ： STM32F405RGT6  
IDE :  Keil  
ST lib ： HAL  

## 文件介绍

* **移植文件**:是STM32上的陀螺仪驱动文件，移植到自己的工程只需要这里面的3个文件。  
* **icm20689移植版本demo**:是一个移植demo，可以参照这个demo配置spi参数以及程序结构。  

[这里我把dmp配置相关代码都封装在了lib里面，如果需要原始版本请发邮件联系我。](adayimaxiga@hotmail.com)

## 使用注意
 
真正需要使用者关注的只有以下几个函数 :   

| 函数 | 功能  | 注意 |
| :------------ |:---------------:| :-----|
|  `icm20689_dmp_setup()`  | 初始化DMP（包括陀螺仪） | 函数内部调用HAL_Delay，因此需要放在SPI及系统时钟之后 |
|  `get_dmp_data()`   | 获取DMP解算出的姿态角，0表示成功，1表示失败  |  这个函数（DMP更新姿态角频率为200Hz，因此读取过快时存在失败的情况。） |
|  `MPU_Get_Accelerometer(short *ax,short *ay,short *az)` | 获取加速度信息        |    注意单位，使用DMP模式初始化，量程必定是+/- 4g |
| `MPU_Get_Gyroscope(short *gx,short *gy,short *gz)` | 获取角速度        |    注意单位，使用DMP模式初始化，量程必定是+/- 2000dps |

## 更新日志

| 更新时间 | 内容  | 注意 |
| :------------ |:---------------:| :-----|
|2019.1.16| 初始版本，测试成功  | 无|
|2019.1.20| lib更新到icm20689_v1.2,对上电漂移5s问题进行了优化  | 无|
## Author Information
**@Name :  LD**  
**@E-mail :  adayimaxiga@hotmail.com**  
**@Wechat :  adayimaxiga**  
**2019.1.16** 
