import sys
sys.path.insert(0, '~/quadcopter/dobby/imu')
sys.path.insert(0, '~/quadcopter/dobby/bmp')

from dobby_imu import MPU9250 as mpu
from dobby_bmp import BMP280 as bmp

class AHRS:

	def __init__(self):
		self.accel_roll = None
		self.accel_pitch = None

	def convert_accel_to_euler(self):
		self.accel_pitch = math.asin(-mpu.accel_data[0]/mpu.g_vector)
		self.accel_roll  = math.asin(mpu.accel_data[1]/(math.cos(self.accel_pitch*mpu.g_vector)))
