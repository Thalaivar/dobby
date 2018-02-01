//******************************************//
//*******		IMU	LIBRARY			********//
//******************************************//

//need to add the flow control, refer to http://www.strawsondesign.com/#!manual-init-cleanup
//and also http://www.strawsondesign.com/#!manual-flow-state

#include <roboticscape.h>
#include <iostream>

typedef enum imu_mode {
			RANDOM_MODE,
			DMP_MODE
	} mode;

class beagle_imu {

	public:
		rc_imu_data_t data;
		rc_imu_config_t config;
		imu_mode mode;

		// sets default configuration for imu //
		void set_configuration_default(rc_imu_config_t* config);

		// set the operation mode for imu //
		void set_operation_mode(imu_mode* mode, imu_mode choice);

		// sampling data in random mode //
		void update_random(rc_imu_data_t* data);

		// call only one step before getting data //
		void init_random(rc_imu_data_t* data, rc_imu_config_t config);

		// enable magnetometer, to be called after set_configuraton_default and before init_random //
		void enable_mag(rc_imu_config_t* config);

		// to print out imu configuration settings //
		void print_config(rc_imu_config_t* config);

		// debug function to read ax, ay, az, gx, gy, gz, mx, my, mz //
		void debug_print_vals(rc_imu_data_t* data);
};
