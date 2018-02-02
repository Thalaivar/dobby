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
sys.path.insert(0, '/home/debian/dobby/imu/')

from dobby_imu import MPU9250

import math
class AHRS(MPU9250):
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

		self.norm_adata = None
	#list of methods in this class
	#self.convert_accel_to_euler()
	#self.convert_gyro_to_euler()
	#self.update_ahrs()

	def convert_accel_to_euler(self):

		self.norm_adata	= math.sqrt((self.accel_data[0]*self.accel_data[0] + self.accel_data[1]*self.accel_data[1] + self.accel_data[2]*self.accel_data[2]))

		self.accel_pitch = -(math.asin(MPU9250.accel_data[0]/self.norm_adata))
		self.accel_roll  = math.asin(MPU9250.accel_data[1]/(self.norm_adata * math.cos(self.accel_pitch)))

		self.accel_pitch = self.accel_pitch * 180 / math.pi
		self.accel_roll  = self.accel_roll  * 180 / math.pi
		# yaw?

	def convert_gyro_to_euler(self):
			gx = self.gyro_data[0]
			gy = self.gyro_data[1]
			gz = self.gyro_data[2]

			#integrate gx gy gz to get euler angles:
			# time step?
#	def low_pass_filter_ahrs(self):

		#implement low pass filter with proper gains for accel and gyro

#	def update_ahrs(self):
			#run update_imu
			#run conversion functions
			#refresh values

			# update_imu()
#			self.convert_accel_to_euler()
#			self.convert_gyro_to_euler()
#			# we can call filtering function to get ahrs_roll ahrs_pitch ahrs_yaw
