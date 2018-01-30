//*************************************************//
//------------- IMU HEADER FILE -------------------//
//*************************************************//

#include "test_imu.h"
	
	//*** remember that imu has two modes of operation, ***//
	//*** random mode and dmp mode						***//			
	//*** in random mode manual call to read data is needed ***//
	//*** in dmp mode it sample automatically at a configurable rate ***//
	//*** configuration (in init function) is also different for botj ***//
	//*** how to implement that here??? ***//
	//*** currently only provison for random mode ***//

void imu::update_random(rc_imu_data_t* data, imu_mode mode) {
		

		if(mode == RANDOM_MODE){
			rc_read_accel_data(&data);
			rc_read_gyro_data(&data);
			rc_read_mag_data(&data);
	}
		
	

void imu::init_random(rc_imu_data_t* data, rc_imu_config_t* config, imu_mode mode) {
		
		rc_set_imu_config_to_defaults(&config);
		rc_initialize_imu(&data, &config);
	}

	//*** how to implement ability to change configuration of imu via something liek params?***//
