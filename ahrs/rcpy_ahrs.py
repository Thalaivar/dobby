import rcpy.mpu9250 as mpu9250

class AHRS:
	def __init__(self):
		self.imu = imu = mpu9250.IMU()

