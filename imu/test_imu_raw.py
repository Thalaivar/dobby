from dobby_imu import MPU9250

imu = MPU9250(MPU9250.ACCEL_4G, MPU9250.GYRO_500DPS, MPU9250.MAG_16BITS, MPU9250.MAG_100_HZ)

for i in range(5000):
	if imu.is_data_ready():
		imu.update()
		print(imu.accel_data)
	
