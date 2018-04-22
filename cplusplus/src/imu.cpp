#include "imu.h"

IMU::IMU(void){

    config.gyro_fsr = G_FSR_1000DPS;
    config.accel_fsr = A_FSR_4G;
    config.gyro_dlpf = GYRO_DLPF_92;
    config.accel_dlpf = ACCEL_DLPF_92;

    config.enable_magnetometer = 1;

    config.dmp_sample_rate = 200;
    config.orientation = ORIENTATION_Z_UP;
    config.compass_time_constant = 2.0;
    config.dmp_interrupt_priority = sched_get_priority_max(SCHED_FIFO)-1;
    config.show_warnings = 0;

    this->is_initialized = false;
    this->is_calibrated = false;
}


int IMU::init_imu(){

    if(this->is_initialized){
      cerr << "IMU already initialized!" << '\n';
      return 0;
    }

    if(rc_is_gyro_calibrated() == 0){
      printf("Gyro needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_calibrate_gyro_routine() < 0) {
        printf("Gyro calibration failed!\n");
        return -1;
      }
    }

    if(rc_is_mag_calibrated() == 0){
      printf("Mag needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_calibrate_mag_routine() < 0) {
        printf("Mag calibration failed!\n");
        return -1;
      }
    }

    if(rc_is_mag_calibrated() == 1 && rc_is_gyro_calibrated() == 1){
      this->is_calibrated = true;
    }

    if(rc_initialize_imu_dmp(&data, config) < 0){
      printf("DMP init failed!\n");
      return -1;
    }

    this->is_initialized = true;
    return 0;
}

void IMU::update(){

  // populate euler angles with latest data in rad
  euler_angles[ROLL]  = data.fused_TaitBryan[ROLL];
  euler_angles[PITCH] = data.fused_TaitBryan[PITCH];
  euler_angles[YAW]   = get_calYaw(data.fused_TaitBryan[YAW]);

  // populate body rates with latest data in rad/s
  body_rates[ROLL]  = data.gyro[ROLL]*DEG_TO_RAD;
  body_rates[PITCH] = data.gyro[PITCH]*DEG_TO_RAD;
  body_rates[YAW]   = data.gyro[YAW]*DEG_TO_RAD;

}

// Adding InitialYaw Function Definitions:
void IMU::set_initialYaw(){
	// Propose to Average Yaw Reading for 10 seconds
	// and set Yaw Angle
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	float sumYaw = 0.0;
	if (this->is_initialized){
		for(int i =0; i < 100; i++){
			sumYaw += data.fused_TaitBryan[YAW];
//			cout << data.fused_TaitBryan[YAW]*RAD_TO_DEG << endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

		}

		this->initialYaw = sumYaw/100.0;

		cout << "Yaw Set to :" << this->initialYaw*RAD_TO_DEG << endl ;

	}
	else {
		cout << "IMU was not initialised. Yaw Angle not Calibrated" << endl;
	}
}

float IMU::get_calYaw(float rawYaw){
	if(rawYaw < this-> initialYaw){
		return rawYaw - this->initialYaw + 360.0f*(1/RAD_TO_DEG);
	}
	else{
		return rawYaw - this->initialYaw ;
	}
}
