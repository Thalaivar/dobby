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
				ahrs.euler_comp_update()
				loop_rate[i] = ahrs.dt_time
				i = i + 1
		except KeyboardInterrupt:
			break
			print("Done!")
	i = 0
	temp = 0

	for i in range(np.size(loop_rate)):
		temp = temp + loop_rate[i]
	
	temp = temp/10000
	print(temp)
if __name__ == '__main__':
	main()
