//******************************************//
//*******		IMU	LIBRARY			********//
//******************************************//

#include <roboticscape.h>
#include <iostream>
class armIMU {

	public:
		rc_imu_data_t data;
		rc_imu_config_t config;
		
		// sets default configuration for imu //
		void set_configuration_default(rc_imu_config_t* config);

		// sampling data in random mode //
		void update_random(rc_imu_data_t* data);

		// call only one step before getting data //
		void init_random(rc_imu_data_t* data, rc_imu_config_t config);
	
		// enable magnetometer, to be called after set_configuraton_default and before init_random //
		void enable_mag(rc_imu_config_t* config);
		
		//to print out imu configuration settings
		void print_config(rc_imu_config_t* config);
};


