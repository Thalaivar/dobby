#include "beagleIMU.h"

beagle_imu imu;

int main() {
		rc_initialize();

		imu.set_configuration_default(&imu.config);
		imu.set_operation_mode(&imu.mode, RANDOM_MODE);

		imu.config.enable_magnetometer = 1;
		imu.init_random(&imu.data, imu.config);

		imu.print_config(&imu.config);

		for(int i = 0; i <= 500; i++) {
			imu.update_random(&imu.data);
			imu.debug_print_vals(&imu.data);
		}

		rc_cleanup();
	}
