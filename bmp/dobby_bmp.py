import smbus
import numpy as np
import math

# main class object has #
# 1. config_class		#
# 2. temperature		#
# 3. pressure			#
# 4. t_fine				#
# 5. altitude			#
#########################

# note : while adding params library, we could make SEA_LEVEL_HPA changeable, need to write func for that #
class BMP280:
	SEA_LEVEL_HPA = 1013.25
	BMP_I2C_BUS = 2

	#	I2C Addresses	#
	__BMP280_ADDRESS               = 0x76
	__BMP280_CHIPID                = 0x58

	# 	Registers	#
	__BMP280_REGISTER_DIG_T1       = 0x88
	__BMP280_REGISTER_DIG_T2       = 0x8A
	__BMP280_REGISTER_DIG_T3       = 0x8C

	__BMP280_REGISTER_DIG_P1       = 0x8E
	__BMP280_REGISTER_DIG_P2       = 0x90
	__BMP280_REGISTER_DIG_P3       = 0x92
	__BMP280_REGISTER_DIG_P4       = 0x94
	__BMP280_REGISTER_DIG_P5       = 0x96
	__BMP280_REGISTER_DIG_P6       = 0x98
	__BMP280_REGISTER_DIG_P7       = 0x9A
	__BMP280_REGISTER_DIG_P8       = 0x9C
	__BMP280_REGISTER_DIG_P9       = 0x9E

	__BMP280_REGISTER_CHIPID       =  0xD0
	__BMP280_REGISTER_VERSION      =  0xD1
	__BMP280_REGISTER_SOFTRESET    =  0xE0

	__BMP280_REGISTER_CAL26        =  0xE1
	__BMP280_REGISTER_CONTROL      =  0xF4
	__BMP280_REGISTER_CONFIG       =  0xF5
	__BMP280_REGISTER_PRESSUREDATA =  0xF7
	__BMP280_REGISTER_TEMPDATA     =  0xFA

	bus = smbus.SMBus(BMP_I2C_BUS)

	def __init__(self):
		self.config_class = self.config_class()
		if self.init_bmp():
			self.temperature = None
			self.t_fine 	 = None
			self.pressure  	 = None
			self.altitude    = None

	## begin low level communication functions ##

	# read8/write8 is write/read_byte_data
	# read16/write16 is read/write_i2c_block_data
	# first param is always __BMP280_ADDRESS

	# second param is Registers

	# third is number of bytes

	def read_byte(self, register):
		data = self.bus.read_byte_data(self.__BMP280_ADDRESS, register)
		return data

	def write_byte(self, register, value):
		self.bus.write_byte_data(self.__BMP280_ADDRESS, register, value)

	def read_16_bits(self, register):
		data = self.bus.read_i2c_block_data(self.__BMP280_ADDRESS, register, 2)
		value = ((data[0] << 8) | data[1])
		return value

	def read_16_bits_signed(self, register):
		return int(self.read_16_bits(register))

	def read_16_bits_LE(self, register):
		temp = self.read_16_bits(register)
		return (temp >> 8) | (temp << 8)

	def read_16_bits_signed_LE(self, register):
		return int(self.read_16_bits_LE(register))

	def read_24_bits(self, register):
		data = self.bus.read_i2c_block_data(self.__BMP280_ADDRESS, register, 3)
		return (data[0] << 8 | data[1] << 8 | data[2])

	## end low level communication functions ##

	def init_bmp(self):
		if self.read_byte(self.__BMP280_REGISTER_CHIPID) != self.__BMP280_CHIPID :
			raise IOError('BMP280 not active!\n')
			return False

		else:
			print "BMP280 sensor detected!\n"
			self.read_coefficients()

			self.write_byte(self.__BMP280_REGISTER_CONTROL, 0x3F)
			return True

	def read_temperature(self):
		adc_t = self.read_24_bits(self.__BMP280_REGISTER_TEMPDATA)
		adc_t >>= 4

		var_1 = (((adc_t>>3) - (int(self.config_class.dig_T1 <<1)))*(int(self.config_class.dig_T2))) >> 11
		var_2 = (((((adc_t>>4) - ((int(self.config_class.dig_T1))))*((adc_t>>4) - (int(self.config_class.dig_T1)))) >> 12)*(int(self.config_class.dig_T3))) >> 14

		#incase you cant find it t_fine is int_32t
		self.t_fine = var_1 + var_2

		self.temperature  = (self.t_fine * 5 + 128) >> 8
		self.temperature = self.temperature/100

	def read_pressure(self):
		self.read_temperature()

		adc_p = self.read_24_bits(self.__BMP280_REGISTER_PRESSUREDATA)
		adc_p >>= 4

		var1 = (int(self.t_fine)) - 128000
		var2 = var1 * var1 * (int(self.config_class.dig_P6))
		var2 = var2 + ((var1*(int(self.config_class.dig_P5)))<<17)
		var2 = var2 + ((int(self.config_class.dig_P4))<<35)
		var1 = ((var1 * var1 * int(self.config_class.dig_P3))>>8) + ((var1 * int(self.config_class.dig_P2))<<12)
		var1 = ((((int(1))<<47) + var1))*(int(self.config_class.dig_P1))>>33

		if var1 == 0:
			return 0

		else:
			self.pressure = 1048576 - adc_p
			self.pressure = (((self.pressure<<31) - var2)*3125) / var1
			var1 = ((int(self.config_class.dig_P9))*(self.pressure>>13)*(self.pressure>>13)) >> 25
			var2 = ((int(self.config_class.dig_P8))*self.pressure) >> 19

			self.pressure = ((self.pressure + var1 + var2) >> 8) + ((int(self.config_class.dig_P7))<<4);

  			float(self.pressure/256)

	def read_altitude(self):

			self.read_pressure()
			self.pressure /= 100

			self.altitude = 44330 * (1.0 - math.pow(self.pressure / self.SEA_LEVEL_HPA, 0.1903))

	class config_class:
		def __init__(self):
			self.dig_T1 = None
			self.dig_T2 = None
			self.dig_T3 = None

			self.dig_P1 = None
			self.dig_P2 = None
			self.dig_P3 = None
			self.dig_P4 = None
			self.dig_P5 = None
			self.dig_P6 = None
			self.dig_P7 = None
			self.dig_P8 = None
			self.dig_P9 = None

			self.dig_H1 = None
			self.dig_H2 = None
			self.dig_H3 = None
			self.dig_H4 = None
			self.dig_H5 = None
			self.dig_H6 = None

	def read_coefficients(self):
		self.config_class.dig_T1 = self.read_16_bits_LE(self.__BMP280_REGISTER_DIG_T1)
		self.config_class.dig_T2 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_T2)
		self.config_class.dig_T3 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_T3)

		self.config_class.dig_P1 = self.read_16_bits_LE(self.__BMP280_REGISTER_DIG_P1)
		self.config_class.dig_P2 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P2)
		self.config_class.dig_P3 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P3)
		self.config_class.dig_P4 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P4)
		self.config_class.dig_P5 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P5)
		self.config_class.dig_P6 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P6)
		self.config_class.dig_P7 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P7)
		self.config_class.dig_P8 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P8)
		self.config_class.dig_P9 = self.read_16_bits_signed_LE(self.__BMP280_REGISTER_DIG_P9)
