#put magdata in mag_data
#put acceldata in accel_data
#put gyrodata in gyro_data
#update raw values
#convert to angles
#filter noise
#provide updated angles
# we can try to create ahrs_update() only
# or we write multiple functions integrate them under ahrs_update and call only this


import sys
sys.path.insert(0, '/home/debian/dobby/imu')
sys.path.insert(0, '/home/debian/dobby/bmp')

import math
class AHRS:
	accel_roll   = None
	accel_pitch  = None
	accel_yaw    = None

	gyro_roll 	 = None
	gyro_pitch   = None
	gyro_yaw	 = None

	mag_roll     = None
	mag_pitch    = None
	mag_yaw      = None

	def __init__(self):
		#we can setup imu here only that will shorten main code

		self.accel_roll = None
		self.accel_pitch = None

	def convert_accel_to_euler(self):
