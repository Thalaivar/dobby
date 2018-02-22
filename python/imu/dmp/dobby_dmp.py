from dmpmap import *
from dmpKey import *
from dmp_firmware import *

class DMP:
	#I2C bus and address definitions for Robotics Cape
	__IMU_ADDR= 0x68
	__IMU_BUS= 2

	# internal DMP sample rate limits
	__DMP_MAX_RATE= 200
	__DMP_MIN_RATE =4
	__IMU_POLL_TIMEOUT= 300 # milliseconds
	__MAX_FIFO_BUFFER=	128


#******************************************************************
# register offsets
#*****************************************************************/
	__SELF_TEST_X_GYRO  = 0x00
	__SELF_TEST_Y_GYRO  = 0x01
	__SELF_TEST_Z_GYRO  = 0x02
	__X_FINE_GAIN       = 0x03
	__Y_FINE_GAIN       = 0x04
	__Z_FINE_GAIN       = 0x05
	__SELF_TEST_X_ACCEL =  0x0D
	__SELF_TEST_Y_ACCEL =  0x0E
	__SELF_TEST_Z_ACCEL =  0x0F
	__SELF_TEST_A       = 0x10
	__XG_OFFSET_H       = 0x13
	__XG_OFFSET_L       = 0x14
	__YG_OFFSET_H       = 0x15
	__YG_OFFSET_L       = 0x16
	__ZG_OFFSET_H       = 0x17
	__ZG_OFFSET_L       = 0x18
	__SMPLRT_DIV        = 0x19
	__CONFIG            = 0x1A
	__GYRO_CONFIG       = 0x1B
	__ACCEL_CONFIG      = 0x1C
	__ACCEL_CONFIG_2    = 0x1D
	__LP_ACCEL_ODR      = 0x1E
	__WOM_THR           = 0x1F
	__MOT_DUR           = 0x20
	__ZMOT_THR          = 0x21
	__ZRMOT_DUR         = 0x22
	__FIFO_EN           = 0x23
	__I2C_MST_CTRL      = 0x24
	__I2C_SLV0_ADDR     = 0x25
	__I2C_SLV0_REG      = 0x26
	__I2C_SLV0_CTRL     = 0x27
	__I2C_SLV1_ADDR     = 0x28
	__I2C_SLV1_REG      = 0x29
	__I2C_SLV1_CTRL     = 0x2A
	__I2C_SLV2_ADDR     = 0x2B
	__I2C_SLV2_REG      = 0x2C
	__I2C_SLV2_CTRL     = 0x2D
	__I2C_SLV3_ADDR     = 0x2E
	__I2C_SLV3_REG      = 0x2F
	__I2C_SLV3_CTRL     = 0x30
	__I2C_SLV4_ADDR     = 0x31
	__I2C_SLV4_REG      = 0x32
	__I2C_SLV4_DO       = 0x33
	__I2C_SLV4_CTRL     = 0x34
	__I2C_SLV4_DI       = 0x35
	__I2C_MST_STATUS    = 0x36
	__INT_PIN_CFG       = 0x37
	__INT_ENABLE        = 0x38
	__DMP_INT_STATUS    = 0x39
	__INT_STATUS        = 0x3A
	__ACCEL_XOUT_H      = 0x3B
	__ACCEL_XOUT_L      = 0x3C
	__ACCEL_YOUT_H      = 0x3D
	__ACCEL_YOUT_L      = 0x3E
	__ACCEL_ZOUT_H      = 0x3F
	__ACCEL_ZOUT_L      = 0x40
	__TEMP_OUT_H        = 0x41
	__TEMP_OUT_L        = 0x42
	__GYRO_XOUT_H       = 0x43
	__GYRO_XOUT_L       = 0x44
	__GYRO_YOUT_H       = 0x45
	__GYRO_YOUT_L       = 0x46
	__GYRO_ZOUT_H       = 0x47
	__GYRO_ZOUT_L       = 0x48
	__EXT_SENS_DATA_00  = 0x49
	__EXT_SENS_DATA_01  = 0x4A
	__EXT_SENS_DATA_02  = 0x4B
	__EXT_SENS_DATA_03  = 0x4C
	__EXT_SENS_DATA_04  = 0x4D
	__EXT_SENS_DATA_05  = 0x4E
	__EXT_SENS_DATA_06  = 0x4F
	__EXT_SENS_DATA_07  = 0x50
	__EXT_SENS_DATA_08  = 0x51
	__EXT_SENS_DATA_09  = 0x52
	__EXT_SENS_DATA_10  = 0x53
	__EXT_SENS_DATA_11  = 0x54
	__EXT_SENS_DATA_12  = 0x55
	__EXT_SENS_DATA_13  = 0x56
	__EXT_SENS_DATA_14  = 0x57
	__EXT_SENS_DATA_15  = 0x58
	__EXT_SENS_DATA_16  = 0x59
	__EXT_SENS_DATA_17  = 0x5A
	__EXT_SENS_DATA_18  = 0x5B
	__EXT_SENS_DATA_19  = 0x5C
	__EXT_SENS_DATA_20  = 0x5D
	__EXT_SENS_DATA_21  = 0x5E
	__EXT_SENS_DATA_22  = 0x5F
	__EXT_SENS_DATA_23  = 0x60
	__MOT_DETECT_STATUS =  0x61
	__I2C_SLV0_DO       = 0x63
	__I2C_SLV1_DO       = 0x64
	__I2C_SLV2_DO       = 0x65
	__I2C_SLV3_DO       = 0x66
	__I2C_MST_DELAY_CTRL = 0x67
	__SIGNAL_PATH_RESET =   0x68
	__MOT_DETECT_CTRL   = 0x69
	__USER_CTRL         = 0x6A
	__PWR_MGMT_1        = 0x6B
	__PWR_MGMT_2        = 0x6C
	__DMP_BANK          = 0x6D
	__DMP_RW_PNT        = 0x6E
	__DMP_REG           = 0x6F
	__DMP_REG_1         = 0x70
	__DMP_REG_2         = 0x71
	__FIFO_COUNTH       = 0x72
	__FIFO_COUNTL       = 0x73
	__FIFO_R_W          = 0x74
	__WHO_AM_I_MPU9250  = 0x75 # Should return 0x71
	__XA_OFFSET_H       = 0x77
	__XA_OFFSET_L       = 0x78
	__YA_OFFSET_H       = 0x7A
	__YA_OFFSET_L       = 0x7B
	__ZA_OFFSET_H       = 0x7D
	__ZA_OFFSET_L       = 0x7E




#*******************************************************************
# CONFIG register bits
#******************************************************************/
	__FIFO_MODE_REPLACE_OLD =	0
	__FIFO_MODE_KEEP_OLD    = 	0x01<<6
	__EXT_SYNC_SET_DISABLE  = 0


#*******************************************************************
# GYRO_CONFIG register bits
#******************************************************************/
	__XGYRO_CTEN		=		0x01<<7
	__YGYRO_CTEN		=		0x01<<6
	__ZGYRO_CTEN		=		0x01<<5
	__GYRO_FSR_CFG_250	=	0x00<<3
	__GYRO_FSR_CFG_500	=	0x01<<3
	__GYRO_FSR_CFG_1000	=	0x02<<3
	__GYRO_FSR_CFG_2000	=	0x03<<3
	__FCHOICE_B_DLPF_EN	=	0x00
	__FCHOICE_B_DLPF_DISABLE = 0x01

#*******************************************************************
# ACCEL_CONFIG register bits
#*****************************************************************/
	__AX_ST_EN			=	0x01<<7
	__AY_ST_EN			=	0x01<<6
	__AZ_ST_EN			=	0x01<<5
	__ACCEL_FSR_CFG_2G	=	0x00<<3
	__ACCEL_FSR_CFG_4G	=	0x01<<3
	__ACCEL_FSR_CFG_8G	=	0x02<<3
	__ACCEL_FSR_CFG_16G	=	0x03<<3

#*******************************************************************
# ACCEL_CONFIG2 register bits
#******************************************************************/
	__ACCEL_FCHOICE_1KHZ =		0x00<<3
	__ACCEL_FCHOICE_4KHZ =		0x01<<3


#*******************************************************************
# INT_PIN_CFG
#******************************************************************/
	__ACTL_ACTIVE_LOW		 =	0x01<<7
	__ACTL_ACTIVE_HIGH		 =	0
	__INT_OPEN_DRAIN		 =	0
	__INT_PUSH_PULL			 =	0x00<<6
	__LATCH_INT_EN			 =	0x01<<5
	__INT_ANYRD_CLEAR		 =	0x01<<4
	__ACTL_FSYNC_ACTIVE_LOW	 =	0x01<<3
	__ACTL_FSYNC_ACTIVE_HIGH =	0x00<<3
	__FSYNC_INT_MODE_EN		 =	0x01<<2
	__FSYNC_INT_MODE_DIS	 =	0x00<<2
	__BYPASS_EN				 =	0x01<<1


#*******************************************************************
#INT_ENABLE register settings
#******************************************************************/
	__WOM_EN			= 		0x01<<6
	__WOM_DIS			=		0x00<<6
	__FIFO_OVERFLOW_EN	=	0x01<<4
	__FIFO_OVERFLOW_DIS	=	0x00<<4
	__FSYNC_INT_EN		=	0x01<<3
	__FSYNC_INT_DIS		=	0x00<<3
	__RAW_RDY_EN		=		0x01
	__RAW_RDY_DIS		=		0x00

#*******************************************************************
# FIFO_EN register settings
#******************************************************************/
	__FIFO_TEMP_EN		=	0x01<<7
	__FIFO_GYRO_X_EN		=	0x01<<6
	__FIFO_GYRO_Y_EN		=	0x01<<5
	__FIFO_GYRO_Z_EN		=	0x01<<4
	__FIFO_ACCEL_EN		=	0x01<<3
	__FIFO_SLV2_EN		=	0x01<<2
	__FIFO_SLV1_EN		=	0x01<<1
	__FIFO_SLV0_EN		=	0x01


#*******************************************************************
# PWR_MGMT_1 register settings
#******************************************************************/
	__H_RESET		=		0x01<<7
	__MPU_SLEEP		=		0x01<<6
	__MPU_CYCLE		=		0x01<<5


#*******************************************************************
# temperature reading constants
#******************************************************************/
	__ROOM_TEMP_OFFSET	=	0x00
	__RTEMP_SENSITIVITY	=	333.87 #// degC/LSB


#*******************************************************************
# USER_CTRL settings bits
#******************************************************************/
	__FIFO_EN_BIT =			0x01<<6
	__I2C_MST_EN  =   		0x01<<5
	__I2C_IF_DIS  =   		0x01<<4
	__FIFO_RST	  =   	0x01<<2
	__I2C_MST_RST =			0x01<<1
	__SIG_COND_RST =			0x01










#******************************************************************
# Magnetometer Registers
#*****************************************************************/
	__AK8963_ADDR	  = 0x0C
	__WHO_AM_I_AK8963 = 0x00 #should return 0x48
	__INFO            = 0x01
	__AK8963_ST1      = 0x02  # data ready status
	__AK8963_XOUT_L   = 0x03  # data
	__AK8963_XOUT_H   = 0x04
	__AK8963_YOUT_L   = 0x05
	__AK8963_YOUT_H   = 0x06
	__AK8963_ZOUT_L   = 0x07
	__AK8963_ZOUT_H   = 0x08
	__AK8963_ST2      = 0x09
	__AK8963_CNTL     = 0x0A  # main mode control register
	__AK8963_ASTC     = 0x0C  # Self test control
	__AK8963_I2CDIS   = 0x0F  # I2C disable
	__AK8963_ASAX     = 0x10  # x-axis sensitivity adjustment value
	__AK8963_ASAY     = 0x11  # y-axis sensitivity adjustment value
	__AK8963_ASAZ     = 0x12  # z-axis sensitivity adjustment value

#******************************************************************
# Magnetometer AK8963_CNTL register Settings
#*****************************************************************/
	__MAG_POWER_DN	=	0x00	# power down magnetometer
	__MAG_SINGLE_MES=	0x01	# powers down after 1 measurement
	__MAG_CONT_MES_1=	0x02	# 8hz continuous self-sampling
	__MAG_CONT_MES_2=	0x06	# 100hz continuous self-sampling
	__MAG_EXT_TRIG	=	0x04	# external trigger mode
	__MAG_SELF_TEST	=	0x08	# self test mode
	__MAG_FUSE_ROM	=	0x0F	# ROM read only mode
	__MSCALE_16		=	0x01<<4
	__MSCALE_14		=	0x00

#******************************************************************
# Magnetometer AK8963_ST2 register definitions
#*****************************************************************/
	__MAGNETOMETER_SATURATION =	0x01<<3

#******************************************************************
# Magnetometer AK8963_ST1 register definitions
#*****************************************************************/
	__MAG_DATA_READY =	0x01

#******************************************************************
# Magnetometer sensitivity in micro Teslas to LSB
#*****************************************************************/
	__MAG_RAW_TO_uT		 = (4912.0/32760.0)
	__BIT_I2C_MST_VDDIO  = (0x80)
	__BIT_FIFO_EN        = (0x40)
	__BIT_DMP_EN         = (0x80)
	__BIT_FIFO_RST       = (0x04)
	__BIT_DMP_RST        = (0x08)
	__BIT_FIFO_OVERFLOW  = (0x10)
	__BIT_DATA_RDY_EN    = (0x01)
	__BIT_DMP_INT_EN     = (0x02)
	__BIT_MOT_INT_EN     = (0x40)
	__BITS_FSR           = (0x18)
	__BITS_LPF           = (0x07)
	__BITS_HPF           = (0x07)
	__BITS_CLK           = (0x07)
	__BIT_FIFO_SIZE_1024 = (0x40)
	__BIT_FIFO_SIZE_2048 = (0x80)
	__BIT_FIFO_SIZE_4096 = (0xC0)
	__BIT_RESET          = (0x80)
	__BIT_SLEEP          = (0x40)
	__BIT_S0_DELAY_EN    = (0x01)
	__BIT_S2_DELAY_EN    = (0x04)
	__BITS_SLAVE_LENGTH  = (0x0F)
	__BIT_SLAVE_BYTE_SW  = (0x40)
	__BIT_SLAVE_GROUP    = (0x10)
	__BIT_SLAVE_EN       = (0x80)
	__BIT_I2C_READ       = (0x80)
	__BITS_I2C_MASTER_DLY= (0x1F)
	__BIT_AUX_IF_EN      = (0x20)
	__BIT_ACTL           = (0x80)
	__BIT_LATCH_EN       = (0x20)
	__BIT_ANY_RD_CLR     = (0x10)
	__BIT_BYPASS_EN      = (0x02)
	__BITS_WOM_EN        = (0xC0)
	__BIT_LPA_CYCLE      = (0x20)
	__BIT_STBY_XA        = (0x20)
	__BIT_STBY_YA        = (0x10)
	__BIT_STBY_ZA        = (0x08)
	__BIT_STBY_XG        = (0x04)
	__BIT_STBY_YG        = (0x02)
	__BIT_STBY_ZG        = (0x01)
	__BIT_STBY_XYZA      = (__BIT_STBY_XA | __BIT_STBY_YA | __BIT_STBY_ZA)
	__BIT_STBY_XYZG      = (__BIT_STBY_XG | __BIT_STBY_YG | __BIT_STBY_ZG)

	__GYRO_SF            = 46850825

	__DMP_FEATURE_TAP            =  (0x001)
	__DMP_FEATURE_ANDROID_ORIENT =  (0x002)
	__DMP_FEATURE_LP_QUAT        =  (0x004)
	__DMP_FEATURE_PEDOMETER      =  (0x008)
	__DMP_FEATURE_6X_LP_QUAT     =  (0x010)
	__DMP_FEATURE_GYRO_CAL       =  (0x020)
	__DMP_FEATURE_SEND_RAW_ACCEL =  (0x040)
	__DMP_FEATURE_SEND_RAW_GYRO  =  (0x080)
	__DMP_FEATURE_SEND_CAL_GYRO  =  (0x100)
	__DMP_FEATURE_SEND_ANY_GYRO  =  (__DMP_FEATURE_SEND_RAW_GYRO |  __DMP_FEATURE_SEND_CAL_GYRO)

	__bus = smbus.SMBus(__IMU_BUS)

	def mpu_write_mem(self, mem_addr, length, data):
		tmp = np.array
