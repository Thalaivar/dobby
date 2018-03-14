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
