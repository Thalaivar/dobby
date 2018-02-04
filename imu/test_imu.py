from dobby_imu import MPU9250

imu = MPU9250()

imu.init_imu()
for i in range(2000):
	if imu.is_data_ready():
		imu.update()
		imu.scale_rawdata()
		print(imu.accel_data)
