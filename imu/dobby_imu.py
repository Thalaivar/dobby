import time
import smbus

# need to add some prints in reset_mpu
# need to add calibration function for mag
# need to add function to change

class MPU9250:
	__AK8963_ADDRESS   =  0x0C<<1
	__AK8963_WHO_AM_I  =  0x00
	__AK8963_INFO      =  0x01
	__AK8963_ST1       =  0x02
	__AK8963_XOUT_L    =  0x03
	__AK8963_XOUT_H    =  0x04
	__AK8963_YOUT_L    =  0x05
	__AK8963_YOUT_H    =  0x06
	__AK8963_ZOUT_L    =  0x07
	__AK8963_ZOUT_H    =  0x08
	__AK8963_ST2       =  0x09
	__AK8963_CNTL      =  0x0A
	__AK8963_ASTC      =  0x0C
	__AK8963_I2CDIS    =  0x0F
	__AK8963_ASAX      =  0x10
	__AK8963_ASAY      =  0x11
	__AK8963_ASAZ      =  0x12
	__SELF_TEST_X_GYRO =  0x00
	__SELF_TEST_Y_GYRO =  0x01
	__SELF_TEST_Z_GYRO =  0x02
	
	#define X_FINE_GAIN      0x03
	#define Y_FINE_GAIN      0x04
	#define Z_FINE_GAIN      0x05
	#define XA_OFFSET_H      0x06
	#define XA_OFFSET_L_TC   0x07
	#define YA_OFFSET_H      0x08
	#define YA_OFFSET_L_TC   0x09
	#define ZA_OFFSET_H      0x0A
	#define ZA_OFFSET_L_TC   0x0B
	
	__SELF_TEST_X_ACCEL = 0x0D
	__SELF_TEST_Y_ACCEL = 0x0E
	__SELF_TEST_Z_ACCEL = 0x0F
	__
	__SELF_TEST_A      =  0x10
	__
	__XG_OFFSET_H      =  0x13
	__XG_OFFSET_L      =  0x14
	__YG_OFFSET_H      =  0x15
	__YG_OFFSET_L      =  0x16
	__ZG_OFFSET_H      =  0x17
	__ZG_OFFSET_L      =  0x18
	__SMPLRT_DIV       =  0x19
	__CONFIG           =  0x1A
	__GYRO_CONFIG      =  0x1B
	__ACCEL_CONFIG     =  0x1C
	__ACCEL_CONFIG2    =  0x1D
	__LP_ACCEL_ODR     =  0x1E
	__WOM_THR          =  0x1F
	__
	__MOT_DUR          =  0x20
	__ZMOT_THR         =  0x21
	__ZRMOT_DUR        =  0x22
	__
	__FIFO_EN          =  0x23
	__I2C_MST_CTRL     =  0x24
	__I2C_SLV0_ADDR    =  0x25
	__I2C_SLV0_REG     =  0x26
	__I2C_SLV0_CTRL    =  0x27
	__I2C_SLV1_ADDR    =  0x28
	__I2C_SLV1_REG     =  0x29
	__I2C_SLV1_CTRL    =  0x2A
	__I2C_SLV2_ADDR    =  0x2B
	__I2C_SLV2_REG     =  0x2C
	__I2C_SLV2_CTRL    =  0x2D
	__I2C_SLV3_ADDR    =  0x2E
	__I2C_SLV3_REG     =  0x2F
	__I2C_SLV3_CTRL    =  0x30
	__I2C_SLV4_ADDR    =  0x31
	__I2C_SLV4_REG     =  0x32
	__I2C_SLV4_DO      =  0x33
	__I2C_SLV4_CTRL    =  0x34
	__I2C_SLV4_DI      =  0x35
	__I2C_MST_STATUS   =  0x36
	__INT_PIN_CFG      =  0x37
	__INT_ENABLE       =  0x38
	__DMP_INT_STATUS   =  0x39
	__INT_STATUS       =  0x3A
	__ACCEL_XOUT_H     =  0x3B
	__ACCEL_XOUT_L     =  0x3C
	__ACCEL_YOUT_H     =  0x3D
	__ACCEL_YOUT_L     =  0x3E
	__ACCEL_ZOUT_H     =  0x3F
	__ACCEL_ZOUT_L     =  0x40
	__TEMP_OUT_H       =  0x41
	__TEMP_OUT_L       =  0x42
	__GYRO_XOUT_H      =  0x43
	__GYRO_XOUT_L      =  0x44
	__GYRO_YOUT_H      =  0x45
	__GYRO_YOUT_L      =  0x46
	__GYRO_ZOUT_H      =  0x47
	__GYRO_ZOUT_L      =  0x48
	__EXT_SENS_DATA_00 =  0x49
	__EXT_SENS_DATA_01 =  0x4A
	__EXT_SENS_DATA_02 =  0x4B
	__EXT_SENS_DATA_03 =  0x4C
	__EXT_SENS_DATA_04 =  0x4D
	__EXT_SENS_DATA_05 =  0x4E
	__EXT_SENS_DATA_06 =  0x4F
	__EXT_SENS_DATA_07 =  0x50
	__EXT_SENS_DATA_08 =  0x51
	__EXT_SENS_DATA_09 =  0x52
	__EXT_SENS_DATA_10 =  0x53
	__EXT_SENS_DATA_11 =  0x54
	__EXT_SENS_DATA_12 =  0x55
	__EXT_SENS_DATA_13 =  0x56
	__EXT_SENS_DATA_14 =  0x57
	__EXT_SENS_DATA_15 =  0x58
	__EXT_SENS_DATA_16 =  0x59
	__EXT_SENS_DATA_17 =  0x5A
	__EXT_SENS_DATA_18 =  0x5B
	__EXT_SENS_DATA_19 =  0x5C
	__EXT_SENS_DATA_20 =  0x5D
	__EXT_SENS_DATA_21 =  0x5E
	__EXT_SENS_DATA_22 =  0x5F
	__EXT_SENS_DATA_23 =  0x60
	__MOT_DETECT_STATUS = 0x61
	__I2C_SLV0_DO      = 0x63
	__I2C_SLV1_DO      = 0x64
	__I2C_SLV2_DO      = 0x65
	__I2C_SLV3_DO      = 0x66
	__I2C_MST_DELAY_CTRL = 0x67
	__SIGNAL_PATH_RESET  = 0x68
	__MOT_DETECT_CTRL  = 0x69
	__USER_CTRL        =  0x6A
	__PWR_MGMT_1       =  0x6B
	__PWR_MGMT_2       =  0x6C
	__DMP_BANK         =  0x6D
	__DMP_RW_PNT       =  0x6E
	__DMP_REG          =  0x6F
	__DMP_REG_1        =  0x70
	__DMP_REG_2        =  0x71
	__FIFO_COUNTH      =  0x72
	__FIFO_COUNTL      =  0x73
	__FIFO_R_W         =  0x74
	__WHO_AM_I_MPU9250 =  0x75
	__XA_OFFSET_H      =  0x77
	__XA_OFFSET_L      =  0x78
	__YA_OFFSET_H      =  0x7A
	__YA_OFFSET_L      =  0x7B
	__ZA_OFFSET_H      =  0x7D
	__ZA_OFFSET_L      =  0x7E
	__MPU9250_ADDRESS  =  0x68
	__
	__MAG_MODE_100 = 0x06
	__MAG_MODE_8 = 0x02
	
	bus = smbus.SMBus(2)
	
	MAGBIAS_X = None
	MAGBIAS_Y = None
	MAGBIAS_Z = None

	def __init__(self, Ascale, Gscale, Mscale, magMode):
		self.a_scale = Ascale
		self.g_scale = Gscale
		self.m_scale = Mscale
		self.mag_mode = magMode
		self.mag_calibration = [0, 0, 0]
		self.mag_bias = [MAGBIAS_X, MAGBIAS_Y, MAGBIAS_Z]
	
	def init_mpu(self):

		# wake up device
		bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0X00)
		time.sleep(0.1)

		# get stable time source
		bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x01)

		bus.write_byte_data(MPU9250_ADDRESS, CONFIG, 0x03)
		bus.write_byte_data(MPU9250_ADDRESS, SMPLRT_DIV, 0x04)

		c = bus.read_byte_Data(MPU9250_ADDRESS, GYRO_CONFIG)
		c = c & ~0x02
		c = c & ~0x18
		c = c | self.g_scale << 3
		bus.write_byte_data(MPU9250_ADDRESS, GYRO_CONFIG, c)

		c = bus.read_byte_data(MPU9250_ADDRESS, ACCEL_CONFIG)
		c = c & ~0x18
		c = c | self.a_scale << 3
		bus.write_byte_data(MPU9250_ADDRESS, ACCEL_CONFIG, c)

		c = bus.read_byte_Data(MPU9250_ADDRESS, ACCEL_CONFIG2)
		c = c & ~0x0F
		c = c | 0x03
		bus.write_byte_data(MPU9250_ADDRESS, ACCEL_CONFIG2, c)

		bus.write_byte_data(MPU9250_ADDRESS, INT_PIN_CFG, 0x22)
		bus.write_byte_data(MPU9250_ADDRESS, INT_ENABLE, 0x01)

	def init_ak8963():

		bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL, 0x00)
		time.sleep(0.01)
		bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL, 0x0F)
		time.sleep(0.01)

		rawData = bus.read_i2c_block_data(AK8963_ADDRESS, AK8963_ASAX, 3)

		for i in range(3):
			self.mag_calibration[i] = (float)(rawData[i] - 128)/256.0 + 1.0

		bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL, 0x00)
		time.sleep(0.1)

		bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL, self.m_scale << 4 | self.mag_mode)
		time.sleep(0.1)

	def reset_mpu():

		bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x80)
		time.sleep(0.1)

	def read_accel(self):

		self.accel_data = bus.read_i2c_block_data(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)

		for i in range(3):
			self.accel_data
