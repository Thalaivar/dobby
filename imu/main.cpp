#include "ARM_IMU.h"

armIMU imu;

int main() {
		rc_initialize();

		imu.set_configuration_default(&imu.config);
		imu.config.enable_magnetometer = 1;
		imu.init_random(&imu.data, imu.config);

		for(int i = 0; i <= 100; i++) {
			imu.update_random(&imu.data);
		}

	}	
