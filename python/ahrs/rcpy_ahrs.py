import rcpy.mpu9250 as mpu9250

class AHRS:
	def __init__(self):
		self.imu = mpu9250
		self.imu.initialize(accel_fsr = mpu9250.ACCEL_FSR_4G,
							gyro_fsr  = mpu9250.GYRO_FSR_500DPS
							enable_magnetometer = True
							orientation = mpu9250.ORIENTATION_Z_DOWN
							compass_time_constant = 2.0
							dmp_interrupt_priority = 
