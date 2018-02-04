from dobby_imu import MPU9250

imu = MPU9250(MPU9250.ACCEL_4G, MPU9250.GYR0_500DPS, MPU9250.MAG_16BITS, MPU9250.MAG_100_HZ)

for i in range(5000):
	imu.update()
	print(imu.accel_data)
	
