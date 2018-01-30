//*************************************************//
//------------- IMU HEADER FILE -------------------//
//*************************************************//

#pragma once

#include <roboticscape.h>

enum imu_mode {
		
		RANDOM_MODE,
		DMP_MODE

	};

class imu {
		
		rc_imu_data_t data;
		rc_imu_config_t config;
		
		private:
			void update_random(rc_imu_data_t* data, imu_mode mode);
			void init_random(rc_imu_data_t * data, rc_imu_config_t* config, imu_mode mode);
		
	};


