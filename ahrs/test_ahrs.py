import sys
sys.path.insert(0, '/home/debian/dobby/imu')
sys.path.insert(0, '/home/debian/dobby/bmp')

from dobby_ahrs import AHRS
from dobby_imu import MPU9250
from dobby_bmp import BMP280


imu = MPU9250(MPU9250.ACCEL_4G, MPU9250.GYRO_250DPS, MPU9250.MAG_16BITS, MPU9250.MAG_100_HZ)
bmp = BMP280()
ahrs = AHRS()
for i in range(2000):
	if imu.is_data_ready():
		imu.update()
		ahrs.convert_accel_to_euler()
	print ahrs.accel_pitch

