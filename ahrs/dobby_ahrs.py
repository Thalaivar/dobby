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


	#list of methods in this class
	self.convert_accel_to_euler()
	self.convert_gyro_to_euler()
	self.update_ahrs()

	def convert_accel_to_euler(self):

		norm_adata = sqrt(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2])

		accel_pitch = (math.asin(accel_data[0]/norm_adata))
		accel_roll  = -math.asin(accel_data[1]/(norm_adata * math.cos(pitch)))

		accel_pitch = accel_pitch * 180 / math.pi
		accel_roll  = accel_roll  * 180 / math.pi
		# yaw?

	def convert_gyro_to_euler(self):
			gx = gyro_data[0]
			gy = gyro_data[1]
			gz = gyro_data[2]

			#integrate gx gy gz to get euler angles:
			# time step?
	def low_pass_filter_ahrs(self):
		#implement low pass filter with proper gains for accel and gyro

	def update_ahrs(self):
			#run update_imu
			#run conversion functions
			#refresh values

			# update_imu()
			self.convert_accel_to_euler()
			self.convert_gyro_to_euler()
			# we can call filtering function to get ahrs_roll ahrs_pitch ahrs_yaw
