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

#ifndef _INV_icm20689_DEFS_H_
#define _INV_icm20689_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* /!\ WARNING: Private header files! Should never be included in an exported header. */

#define HW_ICM6800  0x18
#define HW_ICM20602 0x19
#define HW_ICM20690 0x1a
#define HW_ICM20603 0x1b
#define HW_icm20689 0x1c
#define HW_ICM20609 0x1d

/* compass chip list */
#define HW_AK8963 0x20
#define HW_AK8975 0x21
#define HW_AK8972 0x22
#define HW_AK09911 0x23
#define HW_AK09912 0x24

/* compass I2C address */
#define COMPASS_CHIP_ADDR_DEFAUL 0x0E
#define COMPASS_MAX_RATE 100 // Hz
#define COMPASS_MIN_DLY 10 // ms

#define USE_icm20689 1

#if defined USE_MPU6800
#define MEMS_CHIP  HW_ICM6800
#endif

#if defined USE_ICM20602
#define MEMS_CHIP  HW_ICM20602
#endif

#if defined USE_ICM20690
#define MEMS_CHIP  HW_ICM20690
#endif

#if defined USE_ICM20603
#define MEMS_CHIP  HW_ICM20603
#endif

#if defined USE_icm20689
#define MEMS_CHIP  HW_icm20689
#endif

#if defined USE_ICM20609
#define MEMS_CHIP  HW_ICM20609
#endif

#if !defined(MEMS_CHIP)
    #error "MEMS_CHIP is not defined"
#elif MEMS_CHIP != HW_ICM20602 \
        && MEMS_CHIP != HW_ICM20690 \
        && MEMS_CHIP != HW_ICM20603 \
	&& MEMS_CHIP != HW_icm20689 \
	&& MEMS_CHIP != HW_ICM20609
#error "Unknown value for MEMS_CHIP"
#endif

#define MPU_SUCCESS (0)
#define MPU_COMPASS_NOT_FOUND (int)0x00ABCDEF

#define MSEC_PER_SEC 1000
#define NSEC_PER_MSEC 1000000
#define NSEC_PER_SEC NSEC_PER_MSEC * MSEC_PER_SEC

#define SAMPLE_RATE_DIVIDER 4

#if (MEMS_CHIP == HW_ICM20609 || MEMS_CHIP == HW_icm20689)
#define REG_PRGM_START_ADDRH    (0x70)
#endif

#define REG_BANK_0 0x00
#define REG_BANK_1 0x01

#define DIAMOND_I2C_ADDRESS     0x68
#define BANK_0                  (0 << 7)
#define BANK_1                  (1 << 7)
#define BANK_2                  (2 << 7)
#define BANK_3                  (3 << 7)

#define MPUREG_WHO_AM_I 0x75
#define MPUREG_SMPLRT_DIV 0x19
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_CONFIG 0x1a
#define MPUREG_GYRO_CONFIG 0x1b
#define MPUREG_ACCEL_CONFIG 0x1c
#define MPUREG_ACCEL_CONFIG_2 0x1d
#define MPUREG_LP_CONFIG 0x1e
#define MPUREG_ACCEL_WOM_THR 0x1f
#define MPUREG_ACCEL_WOM_X_THR 0x20
#define MPUREG_ACCEL_WOM_Y_THR 0x21
#define MPUREG_ACCEL_WOM_Z_THR 0x22
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_INT_STATUS 0x3a
#define MPUREG_USER_CTRL 0x6a
#define MPUREG_PWR_MGMT_1 0x6b
#define MPUREG_PWR_MGMT_2 0x6c
#define MPUREG_BANK_SEL 0x6d
#define MPUREG_MEM_START_ADDR 0x6e
#define MPUREG_MEM_R_W 0x6f
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_XA_OFFS_H 0x77
#define MPUREG_YA_OFFS_H 0x7A
#define MPUREG_ZA_OFFS_H 0x7D

#if (MEMS_CHIP == HW_ICM20602)
#define MPUREG_FIFO_WM_INT_STATUS 0x39
#else
#define MPUREG_DMP_INT_STATUS 0x39
#endif

#if (MEMS_CHIP == HW_ICM20690)
#define MPUREG_I2C_SLV0_ADDR 0x25
#define MPUREG_I2C_SLV0_REG  0x26
#define MPUREG_I2C_SLV0_CTRL 0x27
#define MPUREG_I2C_SLV1_ADDR 0x28
#define MPUREG_I2C_SLV1_REG  0x29
#define MPUREG_I2C_SLV1_CTRL 0x2A
#define MPUREG_I2C_SLV2_ADDR 0x2B
#define MPUREG_I2C_SLV2_REG  0x2C
#define MPUREG_I2C_SLV2_CTRL 0x2D
#define MPUREG_I2C_SLV4_CTRL 0x34
#define MPUREG_I2C_SLV0_DO   0x63
#define MPUREG_I2C_SLV1_DO   0x64
#define MPUREG_I2C_SLV2_DO   0x65
#define MPUREG_I2C_MST_DELAY_CTRL 0x67
#endif

#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_TEMP_XOUT_H 0x41
#define MPUREG_TEMP_XOUT_L 0x42
#define MPUREG_GYRO_XOUT_H 0x43
#define MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define MPUREG_GYRO_ZOUT_L 0x48

#if (MEMS_CHIP == HW_ICM20690)
#define MPUREG_EXT_SLV_SENS_DATA_00 0x49
#define MPUREG_EXT_SLV_SENS_DATA_01 0x4A
#define MPUREG_EXT_SLV_SENS_DATA_02 0x4B
#define MPUREG_EXT_SLV_SENS_DATA_03 0x4C
#define MPUREG_EXT_SLV_SENS_DATA_04 0x4D
#define MPUREG_EXT_SLV_SENS_DATA_05 0x4E
#define MPUREG_EXT_SLV_SENS_DATA_06 0x4F
#define MPUREG_EXT_SLV_SENS_DATA_07 0x50
#define MPUREG_EXT_SLV_SENS_DATA_08 0x51
#define MPUREG_EXT_SLV_SENS_DATA_09 0x52
#define MPUREG_EXT_SLV_SENS_DATA_10 0x53
#define MPUREG_EXT_SLV_SENS_DATA_11 0x54
#endif

#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
#define MPUREG_MANUFACTURER_ID 0x00
#define MPUREG_CHIP_ID 0x01
#define MPUREG_REVISION_ID 0x02
#define MPUREG_SELF_TEST_X_GYRO 0x50
#define MPUREG_SELF_TEST_Y_GYRO 0x51
#define MPUREG_SELF_TEST_Z_GYRO 0x52
#else
#define MPUREG_SELF_TEST_X_GYRO 0x00
#define MPUREG_SELF_TEST_Y_GYRO 0x01
#define MPUREG_SELF_TEST_Z_GYRO 0x02
#endif

#define MPUREG_SELF_TEST_X_ACCEL 0x0D
#define MPUREG_SELF_TEST_Y_ACCEL 0x0E
#define MPUREG_SELF_TEST_Z_ACCEL 0x0F

#define MPUREG_SIGNAL_PATH_RESET 0x68
#define MPUREG_ACCEL_INTEL_CTRL  0x69

#if (MEMS_CHIP == HW_ICM20690)
#define MPUREG_ANA_CTRL_NEW_1    0x2E
    #define BIT_ODR_DELAY_TIME_EN   0x80
#define MPUREG_ODR_DLY_CNT_HI    0x5F
#define MPUREG_ODR_DLY_CNT_LO    0x60
#define MPUREG_USER_CTRL_NEW     0x70

// CFG Bank
#define MPUREG_ACC_CTRL_NEW0     0x0D
    #define BIT_ODR_DLY_TIME_EN_CFG     0x40
#define REG_INTOSC_CTRL_NEW     0x12
    #define BIT_DELAY_TIME_RD_DISABLE   0x80
#endif

#if ((MEMS_CHIP != HW_icm20689) && (MEMS_CHIP != HW_ICM20609))
#define MPUREG_CFG_USER_BANK_SEL 0x76
#endif

/*register and associated bit definition*/

/* bank 0 register map */
#define REG_WHO_AM_I            MPUREG_WHO_AM_I

#define REG_USER_CTRL           MPUREG_USER_CTRL
    #if ((MEMS_CHIP == HW_ICM6800)||(MEMS_CHIP == HW_icm20689)||(MEMS_CHIP == HW_ICM20609))
    #define BIT_DMP_RST                     0x08
    #define BIT_DMP_EN                      0x80
    #endif
    #define BIT_FIFO_EN                     0x40
    #if ((MEMS_CHIP != HW_ICM20602)&&(MEMS_CHIP != HW_ICM20603))
    #define BIT_I2C_IF_DIS                  0x10
    #endif
    #if (MEMS_CHIP == HW_ICM20690)
    #define BIT_I2C_MST_EN                  0x20
    #endif
    #define BIT_FIFO_RST                    0x04
    #define BIT_SIG_COND_RESET              0x01

#define REG_LP_CONFIG           MPUREG_LP_CONFIG
    #define BITS_G_AVGCFG                   0x70
    #define BIT_POS_G_AVGCFG                4
    #define BIT_GYRO_CYCLE                  0x80

#define REG_PWR_MGMT_1          MPUREG_PWR_MGMT_1
    #define BIT_DEVICE_RESET                0x80
    #define BIT_SLEEP                       0x40
    #define BIT_CYCLE                       0x20
    #define BIT_GYRO_STDBY                  0x10
    #define BIT_TEMP_DISABLE                0x08
    #define BIT_CLKSEL                      0x07
    #define CLK_SEL                         0x01

#define REG_PWR_MGMT_2          MPUREG_PWR_MGMT_2
    #if ((MEMS_CHIP == HW_ICM6800)||(MEMS_CHIP == HW_icm20689)||(MEMS_CHIP == HW_ICM20609))
    #define BIT_LP_DIS                      0x80
    #define BIT_FIFO_LP_EN                  0x80
    #define BIT_DMP_LP_DIS                  0x40
    #endif
    #if (MEMS_CHIP == HW_ICM20690)
    #define BIT_LP_DIS                      0x80
    #endif
    #define BIT_PWR_ACCEL_STBY              0x38
    #define BIT_PWR_GYRO_STBY               0x07
    #define BIT_PWR_ALL_OFF                 0x3f

#define REG_INT_PIN_CFG         MPUREG_INT_PIN_CFG
    #define BIT_INT_LEVEL                   0x80
    #define BIT_INT_OPEN                    0x40
    #define BIT_LATCH_INT_EN                0x20
    #if (MEMS_CHIP != HW_ICM20690)
    #define BIT_INT_RD_CLEAR                0x10
    #endif
    #define BIT_FSYNC_INT_LEVEL             0x08
    #define BIT_FSYNC_INT_MODE_EN           0x04
    #if ((MEMS_CHIP == HW_icm20689) || (MEMS_CHIP == HW_ICM20609))
	#define BIT_BYPASS_EN               0x02
    #endif
    #if ((MEMS_CHIP != HW_icm20689) || (MEMS_CHIP != HW_ICM20609))
    #define BIT_INT2_EN                     0x01
    #endif

#define REG_INT_ENABLE          MPUREG_INT_ENABLE
    //#if (MEMS_CHIP == HW_icm20689)
    //#define BITS_WOM_INT_EN                 0xE0
    //#else
    #define BIT_WOM_X_INT_EN                0x80
    #define BIT_WOM_Y_INT_EN                0x40
    #define BIT_WOM_Z_INT_EN                0x20
    //#endif
    #if ((MEMS_CHIP == HW_ICM6800)||(MEMS_CHIP == HW_icm20689)||(MEMS_CHIP == HW_ICM20609))
	#define BIT_FIFO_OVERFLOW_EN            0x10
    #define BIT_DMP_INT_EN                  0x02
    #endif
    #define BIT_DATA_RDY_INT_EN             0x01

#define REG_DMP_INT_STATUS      MPUREG_DMP_INT_STATUS
	#define BIT_MSG_DMP_INT                 0x0002
	#define BIT_MSG_DMP_INT_2               0x0400  // CIM Command - SMD
	#define BIT_MSG_DMP_INT_3               0x0800  // CIM Command - Pedometer

#define REG_INT_STATUS          MPUREG_INT_STATUS
    //#if (MEMS_CHIP == HW_icm20689)
    //#define BITS_WOM_INT                    0xE0
    //#else
    #define BIT_WOM_X_INT                   0x80
    #define BIT_WOM_Y_INT                   0x40
    #define BIT_WOM_Z_INT                   0x20
    //#endif
    #define BIT_FIFO_OVERFLOW               0x10
    #if ((MEMS_CHIP == HW_ICM6800)||(MEMS_CHIP == HW_icm20689)||(MEMS_CHIP == HW_ICM20609))
    #define BIT_DMP_INT                     0x02
    #endif
    #define BIT_DATA_RDY_INT                0x01

#define REG_FIFO_EN             MPUREG_FIFO_EN
	#if (MEMS_CHIP == HW_ICM20690)
	#define BIT_TEMP_OUT                        0x80
	#define BIT_GYRO_OUT                        0x70
	#define BIT_ACCEL_OUT                       0x08
	#define BIT_SLV2_OUT                        0x04
	#define BIT_SLV1_OUT                        0x02
	#define BIT_SLV0_OUT                        0x01
	#elif ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
	#define BIT_TEMP_OUT                        0x80
	#define BIT_GYRO_OUT                        0x10
	#define BIT_ACCEL_OUT                       0x08
	#elif ((MEMS_CHIP == HW_ICM6800)||(MEMS_CHIP == HW_icm20689)||(MEMS_CHIP == HW_ICM20609))
	#define BIT_TEMP_OUT                        0x80
	#define BIT_XGYRO_OUT                       0x40
	#define BIT_YGYRO_OUT                       0x20
	#define BIT_ZGYRO_OUT                       0x10
	#define BIT_GYRO_OUT                        0x70
	#define BIT_ACCEL_OUT                       0x08
	#endif

#define REG_FIFO_COUNT_H        MPUREG_FIFO_COUNTH
#define REG_FIFO_R_W            MPUREG_FIFO_R_W

#define REG_MEM_START_ADDR      MPUREG_MEM_START_ADDR
#define REG_MEM_R_W             MPUREG_MEM_R_W
#define REG_MEM_BANK_SEL        MPUREG_BANK_SEL

#define REG_CONFIG              MPUREG_CONFIG
	//#if (MEMS_CHIP != HW_icm20689)
    #define BIT_FIFO_RECORD_MODE            0x80
	//#endif
    #define BIT_FIFO_SNAPSHOT_MODE          0x40
    #define BITS_EXT_SYNC_SET               0x38
	#define BIT_POS_EXT_SYNC_SET            3
	#define BITS_DLPF_CFG                   0x07

#define REG_GYRO_CONFIG        MPUREG_GYRO_CONFIG
    #define BIT_XG_ST                       0x80
    #define BIT_YG_ST                       0x40
    #define BIT_ZG_ST                       0x20
    #if (MEMS_CHIP == HW_ICM20690)
      #define BITS_GYRO_FS_SEL              0x1C
      #define BIT_POS_GYRO_FS_SEL           2
    #else
      #define BITS_GYRO_FS_SEL              0x18
      #define BIT_POS_GYRO_FS_SEL           3
    #endif
    #define BITS_FCHOICE_B                  0x03

#define REG_ACCEL_CONFIG        MPUREG_ACCEL_CONFIG
    #define BIT_XA_ST                       0x80
    #define BIT_YA_ST                       0x40
    #define BIT_ZA_ST                       0x20
    #define BITS_ACCEL_FS_SEL               0x18
    #define BIT_POS_ACCEL_FS_SEL               3
    #if (MEMS_CHIP == HW_ICM20690)
    #define BITS_ACCEL_FS_SEL_OIS           0x07
    #endif

#define REG_ACCEL_CONFIG_2      MPUREG_ACCEL_CONFIG_2
    #if ((MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM6800))
		#define BITS_FIFO_SIZE                  0xC0
    #elif ((MEMS_CHIP == HW_ICM20609)||(MEMS_CHIP == HW_icm20689))
		#define BITS_FIFO_SIZE                  0x00	// 512Bytes FIFO for 20609/Total SRAM 4K
    #endif
    #define BITS_DEC2_CFG                   0x30
    #define BIT_POS_DEC2_CFG                4
    #define BITS_ACCEL_FCHOICE_B            0x08
    #define BITS_A_DLPF_CFG                 0x07

#if ((MEMS_CHIP != HW_icm20689) || (MEMS_CHIP != HW_ICM20609))
#define REG_CFG_USER_BANK_SEL   MPUREG_CFG_USER_BANK_SEL
    #define BIT_USER_BANK                   0x00
    #define BIT_CONFIG_BANK                 0x20
#endif

#define REG_SIGNAL_PATH_RESET   MPUREG_SIGNAL_PATH_RESET
    #if (MEMS_CHIP == HW_ICM20690)
    #define BITS_GYRO_FS_SEL_OIS                 0xE0
    #define BITS_FCHOICE_OIS_B                   0x18
    #define BIT_POS_FCHOICE_OIS_B                   3
    #endif
    #if ((MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
    #define BIT_GYRO_RST_REG                    0x04
	#endif
    #define BIT_ACCEL_RST_REG                   0x02
    #define BIT_TEMP_RST_REG                    0x01

#define REG_ACCEL_INTEL_CTRL    MPUREG_ACCEL_INTEL_CTRL
    #define BIT_ACCEL_INTEL_EN              0x80
    #define BIT_ACCEL_INTEL_MODE            0x40
    #if (MEMS_CHIP == HW_ICM20690)
    #define BITS_ACCEL_FCHOICE_OIS_B             0x30
    #define BIT_POS_ACCEL_FCHOICE_OIS_B             4
    #endif
    //#if (MEMS_CHIP != HW_icm20689)
    #define BIT_WOM_INT_MODE                0x01
	//#endif

#if (MEMS_CHIP == HW_ICM20690)
#define REG_USER_CTRL_NEW       MPUREG_USER_CTRL_NEW
    #define BIT_OIS_ENABLE                  0x02
#define REG_I2C_MST_DELAY_CTRL  MPUREG_I2C_MST_DELAY_CTRL
    #define BIT_SLV0_DLY_EN                 0x01
    #define BIT_SLV1_DLY_EN                 0x02
    #define BIT_SLV2_DLY_EN                 0x04
#endif

/* register for all banks */
#define REG_BANK_SEL            0x7F

/* AUX I2C control definitions */
#define INV_MPU_BIT_SLV_EN      0x80
#define INV_MPU_BIT_BYTE_SW     0x40
#define INV_MPU_BIT_REG_DIS     0x20
#define INV_MPU_BIT_GRP         0x10
#define INV_MPU_BIT_I2C_READ    0x80
    /* data definitions */
#define BYTES_PER_SENSOR         6
#define BYTES_PER_TEMP_SENSOR    2
#define FIFO_COUNT_BYTE          2

#if ((MEMS_CHIP == HW_ICM20609) || (MEMS_CHIP == HW_icm20689))
#define HARDWARE_FIFO_SIZE       512
#else
#define HARDWARE_FIFO_SIZE       1024
#endif

// Fifo size is already define in dmp3Default_20608.c
//#define FIFO_SIZE                (HARDWARE_FIFO_SIZE * 4 / 5)
#define POWER_UP_TIME            100
#define REG_UP_TIME_USEC         100
#define DMP_RESET_TIME           20
#define GYRO_ENGINE_UP_TIME      50
#define MPU_MEM_BANK_SIZE        256
#define IIO_BUFFER_BYTES         8
#define HEADERED_NORMAL_BYTES    8
#define HEADERED_Q_BYTES         16
#define LEFT_OVER_BYTES          128
#define BASE_SAMPLE_RATE         1000L
//#define BASE_SAMPLE_RATE_OIS     8000L

#ifdef FREQ_225
#define MPU_DEFAULT_DMP_FREQ     225
#define PEDOMETER_FREQ           (MPU_DEFAULT_DMP_FREQ >> 2)
#define DEFAULT_ACCEL_GAIN       (33554432L * 5 / 11)
#else
#define MPU_DEFAULT_DMP_FREQ     102
#define PEDOMETER_FREQ           (MPU_DEFAULT_DMP_FREQ >> 1)
#define DEFAULT_ACCEL_GAIN       33554432L
#endif
#define PED_ACCEL_GAIN           67108864L
#define ALPHA_FILL_PED           858993459
#define A_FILL_PED               214748365

#define MIN_MST_ODR_CONFIG       4
#define THREE_AXES               3
#define NINE_ELEM                (THREE_AXES * THREE_AXES)
#define MPU_TEMP_SHIFT           16
#define SOFT_IRON_MATRIX_SIZE    (4 * 9)
#define MAX_5_BIT_VALUE          0x1F
#define BAD_COMPASS_DATA         0x7FFF
#define DEFAULT_BATCH_RATE       400
#define DEFAULT_BATCH_TIME    (MSEC_PER_SEC / DEFAULT_BATCH_RATE)
#define MAX_COMPASS_RATE         115
#define MAX_PRESSURE_RATE        30
#define MAX_ALS_RATE             5
#define DATA_AKM_99_BYTES_DMP  10
#define DATA_AKM_89_BYTES_DMP  9
#define DATA_ALS_BYTES_DMP     8
#define APDS9900_AILTL_REG      0x04
#define BMP280_DIG_T1_LSB_REG                0x88
#define COVARIANCE_SIZE          14
#define ACCEL_COVARIANCE_SIZE  (COVARIANCE_SIZE * sizeof(int))
#define COMPASS_COVARIANCE_SIZE  (COVARIANCE_SIZE * sizeof(int))
#define TEMPERATURE_SCALE  3340827L
#define TEMPERATURE_OFFSET 1376256L
#define SECONDARY_INIT_WAIT 60
#define MPU_SOFT_UPDT_ADDR               0x86
#define MPU_SOFT_UPTD_MASK               0x0F
#define AK99XX_SHIFT                    23
#define AK89XX_SHIFT                    22
#define OPERATE_GYRO_IN_DUTY_CYCLED_MODE       (1<<4)
#define OPERATE_ACCEL_IN_DUTY_CYCLED_MODE      (1<<5)
#define OPERATE_I2C_MASTER_IN_DUTY_CYCLED_MODE (1<<6)
#define ACCEL_DATA_SIZE    6
#define GYRO_DATA_SIZE     6
#define TEMP_DATA_SIZE     2
#define COMPASS_MAX_DATA_SIZE     10

/* this is derived from 1000 divided by 55, which is the pedometer
   running frequency */
#define MS_PER_PED_TICKS         18

/* data limit definitions */
#define MIN_FIFO_RATE            4
#define MAX_FIFO_RATE            MPU_DEFAULT_DMP_FREQ
#define MAX_DMP_OUTPUT_RATE      MPU_DEFAULT_DMP_FREQ
#define MAX_READ_SIZE            128
#define MAX_MPU_MEM              8192
#define MAX_PRS_RATE             281

/* data header defines */
#define PRESSURE_HDR             0x8000
#define ACCEL_HDR                0x4000
#define ACCEL_ACCURACY_HDR       0x4080
#define GYRO_HDR                 0x2000
#define GYRO_ACCURACY_HDR        0x2080
#define COMPASS_HDR              0x1000
#define COMPASS_HDR_2            0x1800
#define CPASS_ACCURACY_HDR       0x1080
#define ALS_HDR                  0x0800
#define SIXQUAT_HDR              0x0400
#define PEDQUAT_HDR              0x0200
#define STEP_DETECTOR_HDR        0x0100

#define COMPASS_CALIB_HDR        0x0080
#define GYRO_CALIB_HDR           0x0040
#define EMPTY_MARKER             0x0020
#define END_MARKER               0x0010
#define NINEQUAT_HDR             0x0008
#define LPQ_HDR                  0x0004

#define STEP_INDICATOR_MASK      0x000f

/* init parameters */
#define MPU_INIT_SMD_THLD        1500
#define MPU_INIT_SENSOR_RATE     5
#define MPU_INIT_GYRO_SCALE      3
#define MPU_INIT_ACCEL_SCALE     0
#define MPU_INIT_PED_INT_THRESH  2
#define MPU_INIT_PED_STEP_THRESH 6
#define COMPASS_SLAVEADDR_AKM_BASE      0x0C
#define COMPASS_SLAVEADDR_AKM           0x0E

#define BIT(x) ( 1 << x )

#define ENABLE  1
#define DISABLE 0

// interrupt configurations related to HW register
#define FSYNC_INT   BIT(7)
#define MOTION_INT  BIT(3)
#define PLL_INT     BIT(2)
#define DMP_INT     BIT(1)
#define I2C_INT     BIT(0)

#define CHIP_AWAKE          (0x01)
#define CHIP_LP_ENABLE      (0x02)

#if ((MEMS_CHIP == HW_ICM6800)||(MEMS_CHIP == HW_icm20689)||(MEMS_CHIP == HW_ICM20609))
#define RST_VAL_PWR_MGMT_1  (0x40)
#else
#define RST_VAL_PWR_MGMT_1  (0x41)
#endif

enum IVORY_SERIAL_INTERFACE {
    SERIAL_INTERFACE_I2C = 1,
    SERIAL_INTERFACE_SPI,
    SERIAL_INTERFACE_INVALID
};

enum mpu_accel_fs {	// In the ACCEL_CONFIG (0x1C) register, the full scale select  bits are :
    MPU_FS_2G = 0,	// 00 = 2G
    MPU_FS_4G,		// 01 = 4
	MPU_FS_8G,		// 10 = 8
    MPU_FS_16G,		// 11 = 16
    NUM_MPU_AFS
};

enum mpu_accel_ois_fs {	// In the ACCEL_CONFIG (0x1C) register, the accel ois full scale select bits are :
    MPU_OIS_FS_2G = 0,	// 00 = 2G
    MPU_OIS_FS_4G,		// 01 = 4
	MPU_OIS_FS_8G,		// 10 = 8
    MPU_OIS_FS_1G,		// 11 = 1
    NUM_MPU_OIS_AFS
};

enum mpu_accel_bw {		// In the ACCEL_CONFIG2 (0x1D) register, the BW setting bits are :
	MPU_ABW_218 = 1,	// 001 = 218 Hz
	MPU_ABW_99,			// 010 = 99 Hz
	MPU_ABW_45,			// 011 = 45 Hz
	MPU_ABW_21,			// 100 = 21 Hz
	MPU_ABW_10,			// 101 = 10 Hz
	MPU_ABW_5,			// 110 = 5 Hz
	MPU_ABW_420,		// 111 = 420 Hz
	NUM_MPU_ABW
};

enum mpu_gyro_fs {		// In the GYRO_CONFIG register, the fS_SEL bits are :
	MPU_FS_250dps = 0,	// 00 = 250
	MPU_FS_500dps,		// 01 = 500
	MPU_FS_1000dps,		// 10 = 1000
	MPU_FS_2000dps,		// 11 = 2000
#if (MEMS_CHIP == HW_ICM20690)
	MPU_FS_31dps = 0x5,	// 101 = 31.25
	MPU_FS_62dps,		// 110 = 62.5
	MPU_FS_125dps,		// 111 = 125
#endif
	NUM_MPU_GFS
};

enum mpu_gyro_bw {   // In the CONFIG register, the  BW setting bits are :
	MPU_GBW_176 = 1, // 001 = 176 Hz
	MPU_GBW_92,		 // 010 = 92 Hz
	MPU_GBW_41,		 // 011 = 41 Hz
	MPU_GBW_20,		 // 100 = 20 Hz
	MPU_GBW_10,		 // 101 = 10 Hz
	MPU_GBW_5,		 // 110 = 5 Hz
    NUM_MPU_GBW
};

/* enum for android sensor*/
enum ANDROID_SENSORS {
	ANDROID_SENSOR_META_DATA = 0,
	ANDROID_SENSOR_ACCELEROMETER,
	ANDROID_SENSOR_GEOMAGNETIC_FIELD,
	ANDROID_SENSOR_ORIENTATION,
	ANDROID_SENSOR_GYROSCOPE,
	ANDROID_SENSOR_LIGHT,
	ANDROID_SENSOR_PRESSURE,
	ANDROID_SENSOR_TEMPERATURE,
	ANDROID_SENSOR_WAKEUP_PROXIMITY,
	ANDROID_SENSOR_GRAVITY,
	ANDROID_SENSOR_LINEAR_ACCELERATION,
	ANDROID_SENSOR_ROTATION_VECTOR,
	ANDROID_SENSOR_HUMIDITY,
	ANDROID_SENSOR_AMBIENT_TEMPERATURE,
	ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
	ANDROID_SENSOR_STEP_DETECTOR,
	ANDROID_SENSOR_STEP_COUNTER,
	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_HEART_RATE,
	ANDROID_SENSOR_PROXIMITY,

	ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
	ANDROID_SENSOR_WAKEUP_ORIENTATION,
	ANDROID_SENSOR_WAKEUP_GYROSCOPE,
	ANDROID_SENSOR_WAKEUP_LIGHT,
	ANDROID_SENSOR_WAKEUP_PRESSURE,
	ANDROID_SENSOR_WAKEUP_GRAVITY,
	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
	ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
	ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
	ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_HEART_RATE,
	ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
	ANDROID_SENSOR_RAW_ACCELEROMETER,
	ANDROID_SENSOR_RAW_GYROSCOPE,
	ANDROID_SENSOR_NUM_MAX,

	ANDROID_SENSOR_B2S,
	ANDROID_SENSOR_FLIP_PICKUP,
	ANDROID_SENSOR_ACTIVITY_CLASSIFICATON,
	ANDROID_SENSOR_SCREEN_ROTATION,
	SELF_TEST,
	SETUP,
	GENERAL_SENSORS_MAX
};

enum SENSOR_ACCURACY {
	SENSOR_ACCEL_ACCURACY = 0,
	SENSOR_GYRO_ACCURACY,
	SENSOR_COMPASS_ACCURACY,
	SENSOR_ACCURACY_NUM_MAX,
};

enum POWER_STATE {
    PowerStateSleepState,
    PowerStateAccLPState,
    PowerStateAccLNState,
    PowerStateGyrLPState,
    PowerState6AxisLPState,
    PowerStateGyrLNState,
    PowerState6AxisLNState
};

#define ODR_MIN_DELAY   5      // 200Hz Limited by DMP limitation
#define ODR_MAX_DELAY   1000   // 1Hz 

#define DEF_ST_ACCEL_FS                 2
#define DEF_ST_GYRO_FS_DPS              250
#define DEF_ST_SCALE                    32768
#define DEF_SELFTEST_GYRO_SENS         (DEF_ST_SCALE / DEF_ST_GYRO_FS_DPS)

/* Helper macros */

#ifndef INV_MIN
#define INV_MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef INV_MAX
#define INV_MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _INV_icm20689_DEFS_H_ */
