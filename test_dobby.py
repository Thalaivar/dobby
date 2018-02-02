from dobby import MPU9250
from dobby import BMP280
from dobby import AHRS

imu = MPU9250(MPU9250.ACCEL_4G, MPU9250.GYRO_250DPS, MPU9250.MAG_16BITS, MPU9250.MAG_100_HZ)
baro = BMP280()
ahrs = AHRS()

for i in range(200):
	imu.update()
	ahrs.convert_accel_to_euler()
	
