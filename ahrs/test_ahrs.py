from dobby_ahrs import AHRS
import sys
sys.path.insert(0, '/home/debian/dobby/imu/')
from dobby_imu import MPU9250

imu = MPU9250()
ahrs = AHRS()

def main():
	imu.init_imu()
	imu.print_bias()
	while(True):
		try:
			if imu.is_data_ready():
				imu.update()
				ahrs.euler_comp_update()
				print(ahrs.euler)

		except KeyboardInterrupt:
			break
			print("Done!")

if __name__ == '__main__':
	main()
