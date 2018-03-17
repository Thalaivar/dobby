#include "imu.h"

IMU::IMU(void){

    config.gyro_fsr = G_FSR_500DPS;
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

void IMU::print_tb_angles(){
    printf("Roll: %f | Pitch: %f | Yaw: %f\n", data.dmp_TaitBryan[1]*RAD_TO_DEG, data.dmp_TaitBryan[0]*RAD_TO_DEG\
                                               , data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
}

void IMU::update(){

  // populate euler angles with latest data
  euler_angles[ROLL]  = data.fused_TaitBryan[IMU_ROLL];
  euler_angles[PITCH] = data.fused_TaitBryan[IMU_PITCH];
  euler_angles[YAW]   = data.fused_TaitBryan[IMU_YAW];

  // populate body rates with latest data
  body_rates[ROLL]  = data.gyro[IMU_ROLL];
  body_rates[PITCH] = data.gyro[IMU_PITCH];
  body_rates[YAW]   = data.gyro[IMU_YAW];

  body_to_euler_rates();

}

void IMU::body_to_euler_rates(){

  // state variables
  float theta, phi, psi;

  // get attitude from imu, it is in terms of euler angles
  phi = euler_angles[ROLL];
  theta = euler_angles[PITCH];
  psi = euler_angles[YAW];

  // get euler rates from body rates
  euler_rates[ROLL] = (cos(psi)/cos(theta))*body_rates[ROLL] - (sin(psi)/cos(theta))*body_rates[PITCH];
  euler_rates[PITCH] = sin(psi)*body_rates[ROLL] + cos(psi)*body_rates[PITCH];
  euler_rates[YAW] = -cos(psi)*tan(theta)*body_rates[ROLL] + sin(psi)*tan(theta)*body_rates[PITCH] + body_rates[YAW];
}
