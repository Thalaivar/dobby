#include "imu.h"

IMU::IMU(void){

    config.gyro_fsr = G_FSR_500DPS;
    config.accel_fsr = A_FSR_4G;
    config.gyro_dlpf = GYRO_DLPF_92;
    config.accel_dlpf = ACCEL_DLPF_92;

    config.enable_magnetometer = 1;

    config.dmp_sample_rate = 200;
    config.orientation = ORIENTATION_Z_DOWN;
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

void IMU::print_tb_angles(){
    printf("Roll: %f | Pitch: %f | Yaw: %f\n", euler_angles[ROLL]*RAD_TO_DEG, euler_angles[PITCH]*RAD_TO_DEG\
                                               , euler_angles[YAW]*RAD_TO_DEG);
}

void IMU::update(){

  // populate euler angles with latest data
  euler_angles[ROLL]  = data.fused_TaitBryan[IMU_ROLL] - initialRoll;
  euler_angles[PITCH] = data.fused_TaitBryan[IMU_PITCH] - initialPitch;
  euler_angles[YAW]   = get_calYaw(data.fused_TaitBryan[IMU_YAW]);

  // populate body rates with latest data
  body_rates[ROLL]  = data.gyro[IMU_ROLL];
  body_rates[PITCH] = data.gyro[IMU_PITCH];
  body_rates[YAW]   = data.gyro[IMU_YAW];

  yaw_rotated_body_rates();

}

void IMU::yaw_rotated_body_rates(){

  // state variables
  float psi;

  // get attitude from imu, it is in terms of euler angles
  psi = euler_angles[YAW];

  // get euler rates from body rates
  body_rates_rotated[ROLL] = cos(psi)*body_rates[ROLL] - sin(psi)*body_rates[PITCH];
  body_rates_rotated[PITCH] = sin(psi)*body_rates[ROLL] + cos(psi)*body_rates[PITCH];
  body_rates_rotated[YAW] = body_rates[YAW];
}

// Adding InitialYaw Function Definitions:
void IMU::set_initialYaw(){
	// Propose to Average Yaw Reading for 10 seconds
	// and set Yaw Angle
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	float sumYaw = 0.0;
	if (this->is_initialized){
		for(int i =0; i < 100; i++){
			sumYaw += data.fused_TaitBryan[IMU_YAW];
//			cout << data.fused_TaitBryan[IMU_YAW]*RAD_TO_DEG << endl;
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

void IMU:: zero_initial_attitude(){
	
	float temp_roll, temp_pitch;
	
	temp_roll = 0;
	temp_pitch = 0;
	
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	if(this->is_initialized) {
	 	for(int i = 0; i < 100; i++){
		 	update();
			temp_roll += data.fused_TaitBryan[IMU_ROLL];
			temp_pitch += data.fused_TaitBryan[IMU_PITCH];
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		 }

		initialRoll   = temp_roll/100.0f;
		initialPitch  = temp_pitch/100.0f;

		cout << "Pitch Set to :" << this->initialRoll*RAD_TO_DEG << endl;
		cout << "Roll Set to :" << this->initialPitch*RAD_TO_DEG << endl;
	 }

	else cout << "Roll and Pitch not calibrated!" << endl;
}
