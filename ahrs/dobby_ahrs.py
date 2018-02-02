import sys
sys.path.insert(0, '/home/debian/dobby/imu')
sys.path.insert(0, '/home/debian/dobby/bmp')

from dobby_imu import MPU9250 as mpu
from dobby_bmp import BMP280 as bmp

import math
class AHRS:

	def __init__(self):
		self.accel_roll = None
		self.accel_pitch = None

	def convert_accel_to_euler(self):
		print(mpu.accel_data)
