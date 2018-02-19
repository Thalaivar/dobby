from dobby_ahrs import AHRS
import sys
sys.path.insert(0, '/home/debian/dobby/imu/')
from dobby_imu import MPU9250
import numpy as np

imu = MPU9250()
ahrs = AHRS()

def main():
	loop_rate = np.zeros((10000,))
	imu.init_imu()
	imu.print_bias()
	i = 0
	while(i < 10000):
		try:
			if imu.is_data_ready():
				imu.update()
				ahrs.euler_dcm_update()
				print(ahrs.euler)
				i = i + 1
		except KeyboardInterrupt:
			print("Done!")
			break
if __name__ == '__main__':
	main()
