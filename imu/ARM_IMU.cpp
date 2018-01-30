#include "ARM_IMU.h"


void armIMU::set_configuration_default(rc_imu_config_t* config){
		
		rc_set_imu_config_to_defaults(config);
	}

void armIMU::update_random(rc_imu_data_t* data){
		
		// some checks in here??
		rc_read_accel_data(data);
		rc_read_gyro_data(data);
		rc_read_mag_data(data);
	}

void armIMU::init_random(rc_imu_data_t* data, rc_imu_config_t config){
		
		//check if init was successful
		printf("Initializing IMU for random mode operation....\n");
		rc_initialize_imu(data, config);
		printf("IMU initialized for random mode!\n");

	}

void armIMU::enable_mag(rc_imu_data_t* config){

		config
	}
